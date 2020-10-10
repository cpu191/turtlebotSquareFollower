% A = TwoDSolver([],[])
%A.IntersectPoint()
classdef TwoDSolver
    properties
        special = zeros(1,2);
        x = zeros(1,4);
        y = zeros(1,4);
        xV = zeros(1,2);
        yV = zeros(1,2)
        m1;%yvec/xvec
        m2;
        b1;
        b2;
        xIn = 0;
        yIn = 0;
        set = 0;
        tole = 0.02;
    end
    methods
        function self = TwoDSolver(a,b,c) %input must be 1x4
            self.set = c;
            if c == 0 % a and b are an array of 4 pts x,y values of 2 lines (2 pts make up a line) a contains only x values and b contains y values
                self.x =a;
                self.y =b;
                if abs(self.x(1,1)- self.x(1,2)) > self.tole && abs(self.y(1,1)- self.y(1,2)) > self.tole
                    self.m1 = (self.y(1,1)- self.y(1,2))/(self.x(1,1)- self.x(1,2));
                    self.b1 = self.y(1,1)-self.x(1,1)*self.m1;
                    self.special(1,1) = 0;
                elseif abs(self.x(1,1)- self.x(1,2)) < self.tole  && abs(self.y(1,1)- self.y(1,2)) > self.tole
                    self.special(1,1) =1;
                end
                if abs(self.x(1,3)- self.x(1,4)) > self.tole  && abs(self.y(1,1)- self.y(1,2)) > self.tole
                    self.m2 = (self.y(1,3)- self.y(1,4))/(self.x(1,3)- self.x(1,4));
                    self.b2 = self.y(1,3)-self.x(1,3)*self.m2;
                    self.special(1,2) = 0;
                elseif abs(self.x(1,3)- self.x(1,4)) < self.tole  && abs(self.y(1,1)- self.y(1,2)) > self.tole
                    self.special(1,2) = 1;
                end
                if abs(self.y(1,1)- self.y(1,2)) < self.tole
                    self.m1 = 0;
                    self.b1 =  y(1,1);
                end
                if abs(self.y(1,3)- self.y(1,4)) < self.tole
                    self.m2 = 0;
                    self.b2 = y(1,3);
                end
            end
            if c == 1 % a,b are arrays containing vector and a pt forming a line
                self.xV(1) = a(3);%pt1
                self.yV(1) = a(4);%pt1
                self.xV(2) = b(3);%pt2
                self.yV(2) = b(4);%pt2
                if abs(a(1)) > self.tole && abs(a(2)) > self.tole
                    self.m1 = a(2)/a(1);
                    self.b1 = self.yV(1)-self.xV(1)*self.m1;
                    self.special(1,1) = 0;
                elseif abs(a(1)) < self.tole  && abs(a(2)) > self.tole
                    self.special(1,1) = 1;
                end
                if abs(b(1)) > self.tole && abs(b(2)) > self.tole
                    self.m2 = b(2)/b(1);
                    self.b2 = self.yV(2)-self.xV(2)*self.m2;
                    self.special(1,2) = 0;
                elseif abs(b(1)) < self.tole  && abs(b(2)) > self.tole
                    self.special(1,2) = 1;
                end
                if abs(a(2)) < self.tole
                    self.m1 = 0;
                    self.b1 =  self.yV(1);
                end
                if abs(b(2)) < self.tole
                    self.m2 = 0;
                    self.b2 = self.yV(2);
                end
            end
        end
        function self = IntersectPoint(self)
            if self.set == 0
                if abs(self.m1-self.m2) > self.tole
                    self.xIn = (self.b2-self.b1)/(self.m1-self.m2); %find the x point
                    self.yIn = self.m1*self.xIn + self.b1;
                end
                if (abs(self.m1-self.m2) < self.tole) && self.special(1,1) == 0 && self.special(1,2) == 0
                    self.xIn  = inf;
                    self.yIn = inf;
                end
                if self.special(1,1) ~= 0 && self.special(1,2) == 0
                    self.xIn = self.x(1,1);
                    self.yIn = self.m2*self.xIn + self.b2;
                elseif self.special(1,1) == 0 && self.special(1,2) ~= 0
                    self.xIn = self.x(1,3);
                    self.yIn = self.m1*self.xIn + self.b1;
                elseif self.special(1,1) ~= 0 && self.special(1,2) ~= 0
                    self.xIn  = inf;
                    self.yIn = inf;
                end
            end
            if self.set == 1
                if self.special(1,1) == 0 && self.special(1,2) == 0
                if abs(self.m1-self.m2) > self.tole
                    self.xIn = (self.b2-self.b1)/(self.m1-self.m2); %find the x point
                    self.yIn = self.m1*self.xIn + self.b1;
                end
                if (abs(self.m1-self.m2) < self.tole) 
                    self.xIn  = inf;
                    self.yIn = inf;
                end
                end
                if self.special(1,1) ~= 0 && self.special(1,2) == 0
                    self.xIn = self.xV(1);
                    self.yIn = self.m2*self.xIn + self.b2;
                elseif self.special(1,1) == 0 && self.special(1,2) ~= 0
                    self.xIn = self.xV(2);
                    self.yIn = self.m1*self.xIn + self.b1;
                elseif self.special(1,1) ~= 0 && self.special(1,2) ~= 0
                    self.xIn  = inf;
                    self.yIn = inf;
                end
            end
        end
        function self = Set(self,x)
            self.set = x;
        end
    end
end
