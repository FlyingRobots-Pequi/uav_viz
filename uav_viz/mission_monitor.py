#!/usr/bin/env python3
"""
UAV Mission Monitor Node

Monitor específico para acompanhar o progresso e status de missões UAV.
Fornece logs detalhados e estatísticas de missão.

Author: UAV Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from uav_interfaces.msg import UavStatus, MissionState, MissionCommand
from uav_interfaces.action import ExecuteMission, NavigationCommand
import time
import json
from datetime import datetime
from typing import Dict, List, Optional


class UAVMissionMonitor(Node):
    """
    Monitor de missões UAV para logging e estatísticas
    """

    def __init__(self):
        super().__init__('uav_mission_monitor')
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado da missão
        self.current_mission = None
        self.mission_history = []
        self.mission_start_time = None
        self.last_position = None
        self.total_distance = 0.0
        
        # Estatísticas
        self.mission_stats = {
            'total_missions': 0,
            'successful_missions': 0,
            'failed_missions': 0,
            'total_flight_time': 0.0,
            'total_distance_flown': 0.0,
            'average_mission_duration': 0.0
        }
        
        # Parâmetros
        self.declare_parameter('log_file_path', '/tmp/uav_mission_log.json')
        self.declare_parameter('save_mission_logs', True)
        
        self.log_file_path = self.get_parameter('log_file_path').value
        self.save_logs = self.get_parameter('save_mission_logs').value
        
        # Subscribers
        self._create_subscribers()
        
        # Publishers
        self.mission_log_pub = self.create_publisher(
            String,
            '/mission_logs',
            self.reliable_qos
        )
        
        # Timer para logs periódicos
        self.create_timer(30.0, self.log_mission_progress)
        
        # Carregar histórico de missões
        self._load_mission_history()
        
        self.get_logger().info('UAV Mission Monitor iniciado')
        
    def _create_subscribers(self):
        """Criar subscribers"""
        self.uav_status_sub = self.create_subscription(
            UavStatus,
            '/uav_status',
            self.uav_status_callback,
            self.reliable_qos
        )
        
        self.mission_state_sub = self.create_subscription(
            MissionState,
            '/mission_state',
            self.mission_state_callback,
            self.reliable_qos
        )
        
        self.mission_cmd_sub = self.create_subscription(
            MissionCommand,
            '/mission_cmd',
            self.mission_command_callback,
            self.reliable_qos
        )
    
    def uav_status_callback(self, msg: UavStatus):
        """Callback para status do UAV"""
        current_pos = (msg.x, msg.y, msg.z)
        
        # Calcular distância percorrida
        if self.last_position and self.current_mission:
            distance = self._calculate_distance(self.last_position, current_pos)
            self.total_distance += distance
        
        self.last_position = current_pos
    
    def mission_state_callback(self, msg: MissionState):
        """Callback para estado da missão"""
        if self.current_mission is None and msg.status == "ONGOING":
            # Nova missão iniciada
            self._start_mission_tracking(msg)
        elif self.current_mission and msg.status in ["SUCCESS", "FAILED"]:
            # Missão finalizada
            self._end_mission_tracking(msg)
        
        # Atualizar estado atual
        if self.current_mission:
            self.current_mission['status'] = msg.status
            self.current_mission['info'] = msg.info
            self.current_mission['last_update'] = datetime.now().isoformat()
    
    def mission_command_callback(self, msg: MissionCommand):
        """Callback para comandos de missão"""
        if self.current_mission:
            # Log do comando
            command_info = {
                'timestamp': datetime.now().isoformat(),
                'command': msg.command,
                'target_position': {
                    'x': msg.target_pose.pose.position.x,
                    'y': msg.target_pose.pose.position.y,
                    'z': msg.target_pose.pose.position.z
                },
                'target_heading': msg.target_heading
            }
            
            self.current_mission['commands'].append(command_info)
            
            self.get_logger().info(
                f"Comando de missão: {msg.command} -> "
                f"({command_info['target_position']['x']:.2f}, "
                f"{command_info['target_position']['y']:.2f}, "
                f"{command_info['target_position']['z']:.2f})"
            )
    
    def _start_mission_tracking(self, mission_state: MissionState):
        """Iniciar rastreamento de uma nova missão"""
        self.mission_start_time = datetime.now()
        self.total_distance = 0.0
        
        self.current_mission = {
            'mission_id': f"mission_{int(time.time())}",
            'start_time': self.mission_start_time.isoformat(),
            'end_time': None,
            'status': mission_state.status,
            'info': mission_state.info,
            'commands': [],
            'distance_flown': 0.0,
            'duration': 0.0,
            'success': False
        }
        
        self.mission_stats['total_missions'] += 1
        
        self.get_logger().info(
            f"🚁 Nova missão iniciada: {self.current_mission['mission_id']}"
        )
        
        # Publicar log
        self._publish_mission_log("MISSION_STARTED", self.current_mission)
    
    def _end_mission_tracking(self, mission_state: MissionState):
        """Finalizar rastreamento da missão atual"""
        if not self.current_mission:
            return
        
        end_time = datetime.now()
        duration = (end_time - self.mission_start_time).total_seconds()
        
        # Atualizar dados da missão
        self.current_mission['end_time'] = end_time.isoformat()
        self.current_mission['duration'] = duration
        self.current_mission['distance_flown'] = self.total_distance
        self.current_mission['status'] = mission_state.status
        self.current_mission['info'] = mission_state.info
        self.current_mission['success'] = (mission_state.status == "SUCCESS")
        
        # Atualizar estatísticas
        if self.current_mission['success']:
            self.mission_stats['successful_missions'] += 1
        else:
            self.mission_stats['failed_missions'] += 1
        
        self.mission_stats['total_flight_time'] += duration
        self.mission_stats['total_distance_flown'] += self.total_distance
        self.mission_stats['average_mission_duration'] = (
            self.mission_stats['total_flight_time'] / 
            self.mission_stats['total_missions']
        )
        
        # Adicionar ao histórico
        self.mission_history.append(self.current_mission.copy())
        
        # Log final
        status_icon = "✅" if self.current_mission['success'] else "❌"
        self.get_logger().info(
            f"{status_icon} Missão finalizada: {self.current_mission['mission_id']} "
            f"- Status: {mission_state.status} "
            f"- Duração: {duration:.1f}s "
            f"- Distância: {self.total_distance:.1f}m"
        )
        
        # Publicar log final
        self._publish_mission_log("MISSION_COMPLETED", self.current_mission)
        
        # Salvar logs se habilitado
        if self.save_logs:
            self._save_mission_logs()
        
        # Reset
        self.current_mission = None
        self.mission_start_time = None
        self.total_distance = 0.0
    
    def log_mission_progress(self):
        """Log periódico do progresso da missão"""
        if not self.current_mission:
            return
        
        duration = (datetime.now() - self.mission_start_time).total_seconds()
        
        progress_info = {
            'mission_id': self.current_mission['mission_id'],
            'duration': duration,
            'distance_flown': self.total_distance,
            'status': self.current_mission['status'],
            'commands_executed': len(self.current_mission['commands'])
        }
        
        self.get_logger().info(
            f"📊 Progresso da missão: {duration:.1f}s, "
            f"{self.total_distance:.1f}m, "
            f"{len(self.current_mission['commands'])} comandos"
        )
        
        self._publish_mission_log("MISSION_PROGRESS", progress_info)
    
    def _calculate_distance(self, pos1: tuple, pos2: tuple) -> float:
        """Calcular distância euclidiana entre duas posições"""
        return ((pos2[0] - pos1[0])**2 + 
                (pos2[1] - pos1[1])**2 + 
                (pos2[2] - pos1[2])**2)**0.5
    
    def _publish_mission_log(self, log_type: str, data: dict):
        """Publicar log de missão"""
        log_msg = {
            'timestamp': datetime.now().isoformat(),
            'type': log_type,
            'data': data
        }
        
        msg = String()
        msg.data = json.dumps(log_msg, indent=2)
        self.mission_log_pub.publish(msg)
    
    def _save_mission_logs(self):
        """Salvar logs de missão em arquivo"""
        try:
            log_data = {
                'mission_history': self.mission_history,
                'statistics': self.mission_stats,
                'last_updated': datetime.now().isoformat()
            }
            
            with open(self.log_file_path, 'w') as f:
                json.dump(log_data, f, indent=2)
            
            self.get_logger().info(f"Logs salvos em: {self.log_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Erro ao salvar logs: {e}")
    
    def _load_mission_history(self):
        """Carregar histórico de missões do arquivo"""
        try:
            with open(self.log_file_path, 'r') as f:
                log_data = json.load(f)
            
            self.mission_history = log_data.get('mission_history', [])
            self.mission_stats = log_data.get('statistics', self.mission_stats)
            
            self.get_logger().info(
                f"Histórico carregado: {len(self.mission_history)} missões, "
                f"{self.mission_stats['successful_missions']} sucessos"
            )
            
        except FileNotFoundError:
            self.get_logger().info("Nenhum histórico anterior encontrado")
        except Exception as e:
            self.get_logger().warn(f"Erro ao carregar histórico: {e}")
    
    def get_mission_statistics(self) -> dict:
        """Obter estatísticas de missão"""
        if self.mission_stats['total_missions'] > 0:
            success_rate = (
                self.mission_stats['successful_missions'] / 
                self.mission_stats['total_missions'] * 100
            )
        else:
            success_rate = 0.0
        
        return {
            **self.mission_stats,
            'success_rate': success_rate,
            'current_mission_active': self.current_mission is not None
        }
    
    def print_statistics(self):
        """Imprimir estatísticas no log"""
        stats = self.get_mission_statistics()
        
        self.get_logger().info("📈 ESTATÍSTICAS DE MISSÃO:")
        self.get_logger().info(f"   Total de missões: {stats['total_missions']}")
        self.get_logger().info(f"   Taxa de sucesso: {stats['success_rate']:.1f}%")
        self.get_logger().info(f"   Tempo total de voo: {stats['total_flight_time']:.1f}s")
        self.get_logger().info(f"   Distância total: {stats['total_distance_flown']:.1f}m")
        self.get_logger().info(f"   Duração média: {stats['average_mission_duration']:.1f}s")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        monitor = UAVMissionMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 Mission Monitor finalizado.")
        if 'monitor' in locals():
            monitor.print_statistics()
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 