function plot_RF(x0,y0,z0,unit_vecto_Ox,unit_vecto_Oy,unit_vecto_Oz,T)

unit_vecto_Ox = T*unit_vecto_Ox;
unit_vecto_Oy = T*unit_vecto_Oy;
unit_vecto_Oz = T*unit_vecto_Oz;
x_Ox = [x0;unit_vecto_Ox(1)];
y_Ox = [y0;unit_vecto_Ox(2)];
z_Ox = [z0; unit_vecto_Ox(3)];
x_Oy = [x0;unit_vecto_Oy(1)];
y_Oy = [y0;unit_vecto_Oy(2)];
z_Oy = [z0;unit_vecto_Oy(3)]; 
x_Oz = [x0;unit_vecto_Oz(1)];
y_Oz = [y0;unit_vecto_Oz(2)];
z_Oz = [z0;unit_vecto_Oz(3)];
plot3(x_Ox,y_Ox,z_Ox,'Color','red');%Ve truc Ox
plot3(x_Oy,y_Oy,z_Oy,'Color','blue');%Ve truc Oy
plot3(x_Oz,y_Oz,z_Oz,'Color','black');%Ve truc Oz
axis([-20 20 -20 20 0 30]);
end

