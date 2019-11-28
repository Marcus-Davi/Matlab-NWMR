clear;clc
laser = rossubscriber('/scan');



while(1)
   scan = receive(laser);
    plot(scan);
end