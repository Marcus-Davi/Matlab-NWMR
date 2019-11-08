function [] = convert2K64F(mat_filename)
data = load(mat_filename);
data = data.traj_final;
x = data(1,:);
y = data(2,:);
theta = data(3,:);
vr = data(4,:);
wr = data(5,:);
t_size = length(x);
output_filename = 'trajK64F.c';

plot(x,y)
xlim([-2 2])
ylim([-2 2])

%% WRITE

file = fopen(output_filename,'w');
fprintf(file,'ref_t REFERENCE = {\n\r');

fprintf(file,'.size = %d, \n\n',t_size);


fprintf(file,'.x = {');
for i=1:t_size
fprintf(file,'%f,',x(i));
end
fprintf(file,'},\n\r');

fprintf(file,'.y = {');
for i=1:t_size
fprintf(file,'%f,',y(i));
end
fprintf(file,'},\n\r');

fprintf(file,'.theta = {');
for i=1:t_size
fprintf(file,'%f,',theta(i));
end
fprintf(file,'},\n\r');

fprintf(file,'.v = {');
for i=1:t_size
fprintf(file,'%f,',vr(i));
end
fprintf(file,'},\n\r');

fprintf(file,'.w = {');
for i=1:t_size
fprintf(file,'%f,',wr(i));
end
fprintf(file,'}\n\r');

fprintf(file,'};\n\r');






end