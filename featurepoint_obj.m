classdef featurepoint_obj <  matlab.mixin.Copyable
	properties (Access = public)
        id=-1
        coor3 = zeros(3,1)    
        %pos_in_frame = containers.Map('KeyType','double','ValueType','any')
        pos_in_frame
    end
    
    methods 
        function obj = featurepoint_obj()
            obj.pos_in_frame=containers.Map('KeyType','double','ValueType','any');
         
            
         end
        function pos=lookup_in_frame(obj, frame_id)
            pos=obj.pos_in_frame(frame_id);
        end
        function obj = initial_point(obj, id, coor3, frames, poses)
            obj.id = id;
            obj.coor3 = coor3;
            
            for i=1:size(frames,1)
                obj.pos_in_frame(frames(i))=poses(i,:)
            end
        end
        function obj=update_coor3(obj, coor3)
            obj.coor3=coor3;
        end
        function obj=update_frame_info(obj, frames, poses)
            for i=1:size(frames,1)
                obj.pos_in_frame(frames(i))=poses(i,:);
            end
        end
        function obj=remove_frame_info(obj, frames, poses)
            for i=1:size(frames,1)
                if isKey(obj.pos_in_frame,frames(i))==1
                    remove(obj.pos_in_frame,frames(i));
                end
            end
        end
        function frames=show_in_frames(obj)
            frames= keys(obj.pos_in_frame);
        end
        
            
    end
   
end