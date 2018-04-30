classdef keyframe_obj < handle
	properties (Access = private)
        id=-1
        set='R'
        point_index=[]
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
        function frame_id=get_id(obj)
            frame_id=obj.id;
        end
        function frame_point_index=get_point_index(obj)
            frame_point_index=obj.point_index;
        end
        function obj = initial_frame(obj, frame_id, frame_set, cur_O, cur_P, frame_point_index)
            obj.id=frame_id;
            obj.set=frame_set;
            obj.O = cur_O;
            obj.R = cur_O';            
            obj.P = cur_P;
            obj.t = -cur_O'*cur_P;
            obj.point_index=frame_point_index;
        end
    end
   
end