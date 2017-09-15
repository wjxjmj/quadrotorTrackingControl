% This source code is written to calculate the angle between two vectors
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL
function ang=angleDelta(p2,p1)
ang=sign(det([[cos(p1);sin(p1)] [cos(p2);sin(p2)]]))*acos(dot([cos(p1);sin(p1)],[cos(p2);sin(p2)]));
end