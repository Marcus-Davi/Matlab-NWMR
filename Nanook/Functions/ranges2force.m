function y = ranges2force(ranges,angleIncremnet,angleMin)
n = length(ranges);
ranges_processed = zeros(n,1);
forces = zeros(2,n);
angle = angleMin;

filter_len = 44;

d0 = 0.1;
d1 = 0.8;
k0 = 1;
for i=1:n
    if(i<filter_len)
        continue
    end
    
if(isnan(ranges(i)))
    %ranges_processed(i) = max;
    ranges_processed(i) = 0;
else
    
    if(ranges(i) > d0 && ranges(i) < d1)
%     ranges_processed(i) = -k0/ranges(i);  
    ranges_processed(i) = -k0/ranges(i);  
    else
    %ranges_processed(i) = ranges(i);  
    end
end

angle = angle + angleIncremnet;

forces(:,i) = ranges_processed(i)*[cos(angle) sin(angle)]';



end



% plot(ranges)
% hold on
% plot(ranges_processed)

Fx = mean(forces(1,:));
Fy = mean(forces(2,:));
Force = [Fx Fy]';
%Angle = atan2(Fy,Fx)

y = Force;
end