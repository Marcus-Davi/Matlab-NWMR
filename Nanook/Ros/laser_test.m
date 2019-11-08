clear;clc
laser = rossubscriber('/scan0');



while(1)
   scan = receive(laser);
    plot(scan);
end