clear all

global pos orient; 
setenv("ROS_DOMAIN_ID","0");

% Cria um nó ROS2
node = ros2node("meu_nodo");
node2 = ros2node("meu_nodo_2");

% Cria um publicador em um tópico (por exemplo, std_msgs/msg/String)
pub = ros2publisher(node, "Topic_gabriel", "geometry_msgs/Twist");
pub2 = ros2publisher(node2, "rosout", "rcl_interfaces/Log");

% % Cria um assinante para receber mensagens desse tópico
% sub = ros2subscriber(node, "chatter", "geometry_msgs/Twist", @minhaCallback);

sub = ros2subscriber(node2, "Topic_gabriel", "geometry_msgs/Twist");

sub2 = ros2subscriber(node2, "rosout", "rcl_interfaces/Log");

msg = ros2message("geometry_msgs/Twist");

    msg.linear.x = 1.0;
    msg.linear.y = 1.0;
    msg.linear.z = 1.0;
    
    msg.angular.x = 2.0;
    msg.angular.y = 2.0;
    msg.angular.z = 2.0;

msg2 = ros2message("rcl_interfaces/Log");
msg2.name = 'Gabriel';

% send(pub2, msg2);
% disp("Msg enviada ");

while(1)
    send(pub2, msg2);
    disp("Msg enviada ");
end

% msg_received = receive(sub2,10);
% disp("Msg recebida")

% 
% 
% Exemplo de função callback
% function minhaCallback(~, msg)
% 
%     if nargin < 2 || isempty(msg)
%         disp("Callback chamada sem mensagem válida.");
%         return;
%     end
% 
%     global pos orient;  % declaração dentro da função
%     pos = [msg.linear.x, msg.linear.y, msg.linear.z];
%     orient = [msg.angular.x, msg.angular.y, msg.angular.z];
%     disp("Recebido: ");
% end

