#!/usr/bin/env python3
"""
UAV Status Monitor Node

Monitor de status independente para UAV que exibe informações em tempo real
no terminal e publica alertas críticos.

Author: UAV Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, BatteryStatus
from uav_interfaces.msg import UavStatus, MissionState
import time
import os


class UAVStatusMonitor(Node):
    """
    Monitor de status UAV para terminal e alertas críticos
    """

    def __init__(self):
        super().__init__('uav_status_monitor')
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Estado atual
        self.uav_status = None
        self.mission_state = None
        self.last_update = None
        self.alert_history = []
        
        # Subscribers
        self._create_subscribers()
        
        # Publishers para alertas
        self.alert_pub = self.create_publisher(
            String,
            '/uav_alerts',
            self.reliable_qos
        )
        
        # Timer para display no terminal
        self.create_timer(2.0, self.display_status)
        
        # Timer para verificação de conexão
        self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info('UAV Status Monitor iniciado')
        
    def _create_subscribers(self):
        """Criar subscribers para dados de status"""
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
    
    def uav_status_callback(self, msg):
        """Callback para status do UAV"""
        self.uav_status = msg
        self.last_update = self.get_clock().now()
        
        # Verificar condições críticas
        self._check_critical_conditions(msg)
    
    def mission_state_callback(self, msg):
        """Callback para estado da missão"""
        self.mission_state = msg
    
    def _check_critical_conditions(self, status):
        """Verificar condições críticas e gerar alertas"""
        alerts = []
        
        # Failsafe ativo
        if status.failsafe:
            alerts.append("FAILSAFE ATIVO!")
        
        # Bateria crítica
        if status.battery_remaining < 0.15:
            alerts.append(f"BATERIA CRÍTICA: {status.battery_remaining*100:.0f}%")
        elif status.battery_remaining < 0.3:
            alerts.append(f"BATERIA BAIXA: {status.battery_remaining*100:.0f}%")
        
        # Veículo não armado durante voo
        if not status.armed and status.is_flying:
            alerts.append("DESARMADO DURANTE VOO!")
        
        # Publicar alertas
        for alert in alerts:
            if alert not in self.alert_history:
                self.alert_history.append(alert)
                alert_msg = String()
                alert_msg.data = alert
                self.alert_pub.publish(alert_msg)
                self.get_logger().warn(alert)
        
        # Limitar histórico de alertas
        if len(self.alert_history) > 50:
            self.alert_history = self.alert_history[-25:]
    
    def display_status(self):
        """Exibir status no terminal com layout em 3 colunas"""
        # Limpar terminal
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 120)
        print("UAV STATUS MONITOR".center(120))
        print("=" * 120)
        
        if self.uav_status is None:
            print("Aguardando dados do UAV...".center(120))
            return
        
        print(f"Última atualização: {time.strftime('%H:%M:%S')}".center(120))
        print()
        
        # Cabeçalhos das colunas
        print(f"{'ESTADO GERAL':<39} │ {'NAVEGAÇÃO & POSIÇÃO':<39} │ {'MISSÃO & ALERTAS':<39}")
        print("─" * 39 + "┼" + "─" * 39 + "┼" + "─" * 39)
        
        # Gerar conteúdo das colunas
        col1_lines = self._get_general_status_col()
        col2_lines = self._get_navigation_status_col()  
        col3_lines = self._get_mission_status_col()
        
        # Imprimir colunas lado a lado
        max_lines = max(len(col1_lines), len(col2_lines), len(col3_lines))
        
        for i in range(max_lines):
            line1 = col1_lines[i] if i < len(col1_lines) else ""
            line2 = col2_lines[i] if i < len(col2_lines) else ""
            line3 = col3_lines[i] if i < len(col3_lines) else ""
            
            print(f"{line1:<39} │ {line2:<39} │ {line3:<39}")
        
        print()
        print("=" * 120)
        print("Pressione Ctrl+C para sair".center(120))
    
    def _get_general_status_col(self):
        """Gerar linhas da coluna de estado geral"""
        status = self.uav_status
        lines = []
        
        # Estado de armamento
        armed_status = "SIM" if status.armed else "NÃO"
        lines.append(f"Armado:             {armed_status}")
        
        # Estado de voo
        flying_status = "SIM" if status.is_flying else "NÃO"
        lines.append(f"Voando:             {flying_status}")
        
        # Modo de navegação
        lines.append(f"Modo:               {status.nav_state_text[:20]}")
        
        # Offboard
        offboard_status = "ATIVO" if status.offboard_enabled else "INATIVO"
        lines.append(f"Offboard:           {offboard_status}")
        
        lines.append("")  # Linha vazia
        
        # Status de segurança
        if status.failsafe:
            lines.append("Status:             FAILSAFE ATIVO!")
        else:
            lines.append("Status:             NORMAL")
        
        # Bateria
        battery_level = f"{status.battery_remaining*100:.0f}%"
        battery_voltage = f"({status.battery_voltage:.1f}V)"
        lines.append(f"Bateria:            {battery_level} {battery_voltage}")
        
        # Nível de bateria visual
        if status.battery_remaining < 0.15:
            lines.append("                    CRÍTICA!")
        elif status.battery_remaining < 0.3:
            lines.append("                    BAIXA")
        else:
            lines.append("")
        
        return lines
    
    def _get_navigation_status_col(self):
        """Gerar linhas da coluna de navegação e posição"""
        status = self.uav_status
        lines = []
        
        # Posição
        lines.append(f"Posição X:          {status.x:.2f} m")
        lines.append(f"Posição Y:          {status.y:.2f} m")
        lines.append(f"Altitude:           {status.z:.2f} m")
        
        lines.append("")  # Linha vazia
        
        # Velocidade
        vel_magnitude = (status.vx**2 + status.vy**2 + status.vz**2)**0.5
        lines.append(f"Velocidade:         {vel_magnitude:.2f} m/s")
        lines.append(f"Vel X:              {status.vx:.2f} m/s")
        lines.append(f"Vel Y:              {status.vy:.2f} m/s")
        lines.append(f"Vel Z:              {status.vz:.2f} m/s")
        
        lines.append("")  # Linha vazia
        
        # Home position
        home_dist = ((status.x - status.home_x)**2 + (status.y - status.home_y)**2)**0.5
        lines.append(f"Dist. Home:         {home_dist:.1f} m")
        
        return lines
    
    def _get_mission_status_col(self):
        """Gerar linhas da coluna de missão e alertas"""
        lines = []
        
        # Informações da missão
        if self.mission_state:
            lines.append(f"Missão:             {self.mission_state.status[:20]}")
            if self.mission_state.info:
                # Quebrar info longa em múltiplas linhas
                info_words = self.mission_state.info.split()
                current_line = "Info: "
                for word in info_words:
                    if len(current_line + word) <= 35:
                        current_line += word + " "
                    else:
                        lines.append(current_line.strip())
                        current_line = "      " + word + " "
                if current_line.strip() != "Info:":
                    lines.append(current_line.strip())
        else:
            lines.append("Missão:             Nenhuma")
        
        lines.append("")  # Linha vazia
        
        # Alertas recentes
        if self.alert_history:
            lines.append("ALERTAS RECENTES:")
            for alert in self.alert_history[-4:]:  # Últimos 4 alertas
                # Remover emojis e limitar tamanho
                clean_alert = alert
                for emoji in ["🚨", "🔋", "⚠️", "🛑", "❌", "✅"]:
                    clean_alert = clean_alert.replace(emoji, "")
                clean_alert = clean_alert.strip()
                
                if len(clean_alert) > 35:
                    clean_alert = clean_alert[:32] + "..."
                lines.append(f"- {clean_alert}")
        else:
            lines.append("Alertas:            Nenhum")
        
        return lines
    
    def _get_battery_icon(self, level):
        """Obter ícone da bateria baseado no nível"""
        if level > 0.75:
            return "🔋"
        elif level > 0.50:
            return "🔋"
        elif level > 0.25:
            return "🪫"
        else:
            return "🚨"
    
    def check_connection(self):
        """Verificar conexão com o UAV"""
        if self.last_update is None:
            return
        
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_update).nanoseconds / 1e9
        
        if time_diff > 10.0:  # Sem dados por mais de 10 segundos
            alert = "CONEXÃO PERDIDA COM UAV!"
            if alert not in self.alert_history:
                self.alert_history.append(alert)
                alert_msg = String()
                alert_msg.data = alert
                self.alert_pub.publish(alert_msg)
                self.get_logger().error(alert)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        monitor = UAVStatusMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\nMonitor de Status UAV finalizado.")
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 