# UAV Viz - Sistema de Monitoramento e Visualização UAV

Sistema abrangente de monitoramento e visualização em tempo real para operações de drones PX4 com ROS2.

## Visão Geral

O `uav_viz` é um pacote ROS2 que fornece um sistema completo de monitoramento para UAVs, incluindo:

- 🎯 **Visualização em tempo real** no RViz com pose, trajetória e status
- 📊 **Monitoramento de status** com alertas de segurança
- 🗺️ **Rastreamento de missão** com estatísticas detalhadas
- 🔋 **Indicadores visuais** para bateria, modo de voo e alertas
- 📈 **Logging e análise** de performance de missões

## Características Principais

### UAV Visualizer (`uav_visualizer`)
- Visualização da pose do veículo com eixos de orientação
- Trilha de voo colorida (verde para veículo, amarelo para setpoints)
- Indicadores de status (armado, modo, bateria)
- Vetores de velocidade em tempo real
- Marcadores de posição home e alertas de emergência
- Transformações TF para integração com outros nós

### Status Monitor (`status_monitor`)
- Interface de terminal interativa com status em tempo real
- Alertas automáticos para condições críticas
- Monitoramento de conectividade
- Publicação de alertas no tópico `/uav_alerts`

### Mission Monitor (`mission_monitor`)
- Rastreamento completo de missões com logging JSON
- Estatísticas de performance (duração, distância, taxa de sucesso)
- Histórico persistente de missões
- Logs detalhados de comandos executados

## Instalação

### Pré-requisitos
```bash
# ROS2 Humble ou superior
# PX4 Autopilot com DDS configurado
# Pacotes de dependência
sudo apt install ros-humble-visualization-msgs ros-humble-tf2-ros
```

### Build do Pacote
```bash
cd ~/ros2_workspace/src
# O pacote já está na estrutura do projeto
cd ..
colcon build --packages-select uav_viz
source install/setup.bash
```

## Uso

### Lançamento Completo do Sistema
```bash
# Sistema completo com RViz
ros2 launch uav_viz uav_monitoring.launch.py

# Apenas visualizador (sem RViz)
ros2 launch uav_viz uav_monitoring.launch.py launch_rviz:=false

# Com namespace personalizado
ros2 launch uav_viz uav_monitoring.launch.py namespace:=uav1
```

### Componentes Individuais

#### Visualizador Principal
```bash
ros2 run uav_viz uav_visualizer
```

#### Monitor de Status (Terminal)
```bash
ros2 run uav_viz status_monitor
```

#### Monitor de Missão
```bash
ros2 run uav_viz mission_monitor
```

### Configuração Personalizada
```bash
# Com arquivo de parâmetros personalizado
ros2 launch uav_viz uav_monitoring.launch.py params_file:=/path/to/custom_params.yaml

# Para simulação
ros2 launch uav_viz uav_monitoring.launch.py use_sim_time:=true
```

## Tópicos ROS2

### Tópicos Subscritos
- `/fmu/out/vehicle_local_position` - Posição local do veículo (PX4)
- `/fmu/out/vehicle_attitude` - Orientação do veículo (PX4)
- `/fmu/out/battery_status` - Status da bateria (PX4)
- `/fmu/in/trajectory_setpoint` - Setpoints de trajetória (PX4)
- `/uav_status` - Status customizado do UAV
- `/mission_state` - Estado da missão
- `/mission_cmd` - Comandos de missão

### Tópicos Publicados
- `/uav_viz/vehicle_pose` - Pose atual do veículo
- `/uav_viz/vehicle_path` - Trilha de voo do veículo
- `/uav_viz/setpoint_path` - Trilha dos setpoints
- `/uav_viz/status_markers` - Marcadores de status visual
- `/uav_viz/velocity_vector` - Vetor de velocidade
- `/uav_viz/mission_markers` - Marcadores de missão
- `/uav_viz/alert_markers` - Alertas visuais
- `/uav_alerts` - Alertas de texto
- `/mission_logs` - Logs de missão

## Configuração

### Parâmetros Principais (`config/uav_viz_params.yaml`)

```yaml
uav_visualizer:
  ros__parameters:
    # Configurações de visualização
    vehicle_trail_size: 1000      # Tamanho da trilha
    update_rate: 20.0             # Taxa de atualização (Hz)
    base_frame: "map"             # Frame de referência
    
    # Limites de bateria
    battery_warning_threshold: 0.3    # 30%
    battery_critical_threshold: 0.15  # 15%
    
    # Opções de display
    show_velocity_vector: true
    show_battery_status: true
    show_home_position: true
```

### Cores e Aparência
- **Verde**: Trilha do veículo e status normal
- **Amarelo**: Trilha dos setpoints e vetores de velocidade
- **Laranja**: Avisos (bateria baixa)
- **Vermelho**: Alertas críticos (emergência, failsafe)
- **Azul**: Posição home e veículo desarmado

## Configuração do RViz

O pacote inclui uma configuração personalizada do RViz (`config/uav_monitoring.rviz`) com:

- **Grid de referência** com escala apropriada para voos
- **Pose do veículo** com eixos de orientação
- **Trilhas de voo** diferenciadas por cores
- **Indicadores de status** integrados
- **Visualizações de missão** e alertas
- **Frames salvos** para diferentes perspectivas de visualização

### Frames de Visualização Salvos
- **Wide View**: Vista ampla para missões longas
- **Follow UAV**: Câmera que segue o UAV automaticamente

## Integração com Outros Sistemas

### PX4 Autopilot
O sistema é totalmente compatível com:
- PX4 v1.15 com DDS habilitado
- Micro XRCE-DDS Agent
- QGroundControl para controle manual

### Pacotes UAV Customizados
Integração nativa com:
- `uav_interfaces` - Mensagens customizadas
- `uav_offboard` - Sistema de controle offboard
- `uav_mission` - Gerenciamento de missões

## Exemplos de Uso

### Monitoramento de Voo de Teste
```bash
# Terminal 1: Lançar sistema de monitoramento
ros2 launch uav_viz uav_monitoring.launch.py

# Terminal 2: Monitor de status em tempo real
ros2 run uav_viz status_monitor

# Terminal 3: Controle do UAV (exemplo)
ros2 run uav_offboard flight_manager
```

### Análise de Missão Completa
```bash
# Lançar com logging habilitado
ros2 run uav_viz mission_monitor

# Verificar logs salvos
cat /tmp/uav_mission_log.json
```

### Debugging e Desenvolvimento
```bash
# Modo debug com logs detalhados
ros2 launch uav_viz uav_monitoring.launch.py log_level:=debug

# Verificar tópicos publicados
ros2 topic list | grep uav_viz

# Monitorar alertas
ros2 topic echo /uav_alerts
```

## Troubleshooting

### Problemas Comuns

**RViz não mostra dados:**
- Verificar se o frame `map` está sendo publicado
- Confirmar que os tópicos PX4 estão ativos
- Verificar conexão com Micro XRCE-DDS Agent

**Visualização fora de escala:**
- Ajustar parâmetro `marker_scale` no arquivo de configuração
- Verificar unidades dos dados de posição (metros vs outros)

**Alertas não aparecem:**
- Confirmar que `/uav_status` está sendo publicado
- Verificar thresholds de bateria na configuração

### Logs Úteis
```bash
# Verificar status dos nós
ros2 node list | grep uav

# Logs do visualizador
ros2 log info /uav_visualizer

# Verificar transformações
ros2 run tf2_tools view_frames
```

## Desenvolvimento

### Estrutura do Código
```
uav_viz/
├── uav_viz/
│   ├── uav_visualizer.py      # Visualizador principal
│   ├── status_monitor.py      # Monitor de status
│   ├── mission_monitor.py     # Monitor de missão
│   └── __init__.py
├── config/
│   ├── uav_monitoring.rviz    # Configuração RViz
│   └── uav_viz_params.yaml    # Parâmetros
├── launch/
│   └── uav_monitoring.launch.py
└── README.md
```

### Contribuições
Para contribuir com o projeto:
1. Fork o repositório
2. Crie uma branch para sua feature
3. Implemente testes para novas funcionalidades
4. Submeta um Pull Request

## Licença

MIT License - Veja arquivo LICENSE para detalhes.

## Autores

- UAV Team
- Baseado em px4-offboard de Jaeyoung Lim

## Changelog

### v1.0.0
- Sistema completo de visualização UAV
- Integração com PX4 e interfaces customizadas
- Monitoramento de status e missões
- Configuração RViz personalizada 