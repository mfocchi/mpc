

%if out >0 p2 is right of p1-p0 line (arrow in p1)
function out =Point2isRightOfLine(p0, p1, p2)

%p2 test
  out = (p2(1) - p0(1)) * (p1(2) - p0(2)) - (p1(1) - p0(1)) * (p2(2) - p0(2));
end
