   


 function  out = computeDynamics(n)
A = evalin('base','A');
B = evalin('base','B');
 x = evalin('base','x');
 u = evalin('base','u');
 

   if (n>2)
       out = A*computeDynamics(n-1) + B*u(n-1);
   else
        out = A*x(:,1) + B*u(1);
   end
   
 end