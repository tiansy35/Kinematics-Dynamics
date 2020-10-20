function dy = odefun(t,y)

dy = zeros(4,1);
dy(1) = y(2);
dy(2) = -(y(2)^2*sin(y(3)) - 20*cos(y(1)) + y(4)^2*sin(y(3)) + 10*cos(y(1) + y(3))*cos(y(3)) + 2*y(2)*y(4)*sin(y(3)) + y(2)^2*cos(y(3))*sin(y(3)))/(cos(y(3))^2 - 2);
dy(3) = y(4);
dy(4) = (20*cos(y(1) + y(3)) - 20*cos(y(1)) + 3*y(2)^2*sin(y(3)) + y(4)^2*sin(y(3)) - 20*cos(y(1))*cos(y(3)) + 10*cos(y(1) + y(3))*cos(y(3)) + 2*y(2)*y(4)*sin(y(3)) + 2*y(2)^2*cos(y(3))*sin(y(3)) + y(4)^2*cos(y(3))*sin(y(3)) + 2*y(2)*y(4)*cos(y(3))*sin(y(3)))/(cos(y(3))^2 - 2);

end