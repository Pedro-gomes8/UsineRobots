setenv('ROS_IP', '192.168.23.18');           % Seu IP na rede
setenv('ROS_MASTER_URI', 'http://192.168.23.29:11311');   % IP do ROS Master remoto

rosinit('http://192.168.23.29:11311')

