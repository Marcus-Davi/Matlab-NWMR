function fused_angle = simpleFusion(Mag_yaw,Gyro_vel,Ts)
K = [0.1234 -0.0566]; %Kalman Gain
persistent yaw bias
if(isempty(yaw))
    yaw = 0;
    bias = 0;
end

yaw = yaw - Gyro_vel*Ts - bias*Ts; %sinal trocado só por consistencia na integração

error = Mag_yaw-wrapTo2Pi(yaw); 
	if(abs(error) > pi)

		 if(error > 0.0)
		 error = error - 2*pi;
		  else
		error = error + 2*pi;
         end
         
    end
yaw = yaw + K(1)*error;
bias = bias + K(2)*error;
  fused_angle = wrapTo2Pi(yaw);
end