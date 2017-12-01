classdef ContactArm < HebiArm
    properties
        
    end
    
    methods
        function this = ContactArm(varargin)
            this@HebiArm(varargin{:});
        end
        
        function contact = isContact(this, threshold)
            contact = any(abs(this.getTorqueError) > threshold);
        end
        
        function torqueError = getTorqueError(this)
            fbk = this.group.getNextFeedback();
            expTorque = this.kin.getGravCompTorques(fbk.position, ...
                                                    this.gravityVector);
            measTorque = fbk.torque;
            torqueError = measTorque - expTorque;

        end
    end
end
