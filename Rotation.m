classdef Rotation
    properties
        Quat = quaternion([0 0 0 0]);%w,x,y,z
        Vecbot= zeros(1,3);
        VecTar= zeros(1,3);
        AnVel = 0;
        ThetainRad;
        ti= 0;
        posiRot = 0;
    end
    methods
        function self = Rotation(quat,veta,anvel)
            self.Quat = quat;
            eulXYZ = quat2eul(self.Quat,'XYZ');
            self.Vecbot = [cos(eulXYZ(1,3)) sin(eulXYZ(1,3)) 0];
            self.VecTar = veta;
            self.AnVel = anvel;
        end
        function self = setVecTar(self,veta)
            self.Vectar = veta;
        end
        function self = setVecbot(self,quat)
            self.Quat = quat;
            eulXYZ = quat2eul(quat,'XYZ');
            self.Vecbot = [cos(eulXYZ(1,3)) sin(eulXYZ(1,3)) 0];
        end
        function self = TimeDeter(self)
            Cr = cross(self.Vecbot,self.VecTar);
            if Cr(1,3) > 0
                self.posiRot = 1;
                TDVecbot = [self.Vecbot(1,1) self.Vecbot(1,2)];
                TDTage = [self.VecTar(1,1) self.VecTar(1,2)];
                CosTheta = max(min(dot(TDVecbot,TDTage)/(norm(TDVecbot)*norm(TDTage)),1),-1);
                self.ThetainRad = real(acos(CosTheta));
                self.ti = self.ThetainRad/self.AnVel;
            elseif Cr(1,3) < 0
                self.posiRot = 0;
                TDVecbot = [self.Vecbot(1,1) self.Vecbot(1,2)];
                TDTage = [self.VecTar(1,1) self.VecTar(1,2)];
                CosTheta = max(min(dot(TDVecbot,TDTage)/(norm(TDVecbot)*norm(TDTage)),1),-1);
                self.ThetainRad = -real(acos(CosTheta));
                self.ti = self.ThetainRad/(self.AnVel);
            end
        end
    end
end
% Cr = cross(Vecbot, VecTar);
% if(Cr(1,3) > 0)
%     TDVecbot = [Vecbot(1,1) Vecbot(1,2)];
%     TDTage = [VecTar(1,1) VecTar(1,2)];
%     CosTheta = max(min(dot(TDVecbot,TDTage)/(norm(TDVecbot)*norm(TDTage)),1),-1);
%     Theta = real(acos(CosTheta));
%     msg.Angular.Z = AnVel;%pi/18 pi/12
%     t = Theta/abs(msg.Angular.Z);
%     tic
%     while(1)
%         send(pub,msg);
%         if toc >= (t+ToleTime) %(9 seconds ---> 8.236375 or 8.236373 or =9-0.763627)
%             msg.Angular.Z = 0;
%            send(pub,msg);
%             break;
%         end
%     end
% elseif Cr(1,3) < 0
%     TDVecbot = [Vecbot(1,1) Vecbot(1,2)];
%     TDTage = [VecTar(1,1) VecTar(1,2)];
%     CosTheta = max(min(dot(TDVecbot,TDTage)/(norm(TDVecbot)*norm(TDTage)),1),-1);
%     Theta = real(acos(CosTheta));
%     msg.Angular.Z = -AnVel;%pi/18 pi/12
%     t = Theta/abs(msg.Angular.Z);
%     tic
%     while(1)
%         send(pub,msg);
%         if toc >= (t+ToleTime) %(9 seconds ---> 8.236375 or .236373 or =9-0.763627)
%            msg.Angular.Z = 0;
%            send(pub,msg);
%             break;
%         end
%     end
% end