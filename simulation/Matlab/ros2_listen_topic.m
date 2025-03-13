clear all

% global pos orient; 
setenv("ROS_DOMAIN_ID","0");

% Cria um nó ROS2
node = ros2node("meu_nodo");

sub = ros2subscriber(node, '/cmd_vel', 'geometry_msgs/Twist');

msg = receive(sub, 10);  % Aguarda até 10 segundos para receber uma mensagem
disp(msg.data)           % Exibe o conteúdo da mensagem (no caso de std_msgs/msg/String)
