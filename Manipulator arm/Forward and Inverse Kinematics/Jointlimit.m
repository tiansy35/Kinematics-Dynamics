function [q] = Jointlimit(Q)
%
%   
Qnew = Q;
for i = 1:4
    if (Q(5-i,1)>=-1.4) && (Q(5-i,1)<=1.4)
        if (Q(5-i,2)>=-1.2) && (Q(5-i,2)<=1.4)
            if (Q(5-i,3)>=-1.8) && (Q(5-i,3)<=1.7)
                if (Q(5-i,4)>=-1.9) && (Q(5-i,4)<=1.7)
                    if (Q(5-i,5)>=-2.0) && (Q(5-i,5)<=1.5)
                    else 
                        Qnew(5-i,:)=[];
                    end
                else
                    Qnew(5-i,:)=[];
                end
            else
                Qnew(5-i,:)=[];
            end
        else
            Qnew(5-i,:)=[];
        end
    else
        Qnew(5-i,:)=[];
    end
end
q = Qnew;

end

