map_topic = rossubscriber('map');
map = receive(map_topic);
map_matlab = readBinaryOccupancyGrid(map);
show(map_matlab)