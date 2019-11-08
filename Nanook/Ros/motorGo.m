
function motorGo(pub,v,w)
msg = rosmessage(pub);
msg.Linear.X = v;
msg.Angular.Z = w;
send(pub,msg);
end