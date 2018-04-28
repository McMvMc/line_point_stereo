classdef camera_obj < handle
	properties (Access = private)
        R = eye(3)
        t = [0; 0; 0]
        O = eye(3)
        P = [0; 0; 0]
    end
    methods 
        function cur_R = get_R(obj)
            cur_R = obj.R;
        end
        function cur_t = get_T(obj)
            cur_t = obj.t;
        end
        function cur_O = get_O(obj)
            cur_O = obj.O;
        end
        function cur_P = get_P(obj)
            cur_P = obj.P;
        end
        function obj = set_OP(obj, cur_O, cur_P)
            obj.O = cur_O;
            obj.R = cur_O';
            
            obj.P = cur_P;
            obj.t = -cur_O'*cur_P;
        end
    end
   
end