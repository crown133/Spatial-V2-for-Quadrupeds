function  fh = fhan(x1, x2, r, h)
     d = r*h;
     d0 = h*d;
     y = x1 + h*x2;
     a0 = sqrt(d*d +8*r*abs(y));
     
     if abs(y)>d0
         a = x2 + (a0 - d)*sign(y)/2;
     else 
         a = x2 + y/h;
     end
     
     if abs(a)>d
         fh = -r*sign(a);
     else
         fh = -r*a/d;
     end
end

