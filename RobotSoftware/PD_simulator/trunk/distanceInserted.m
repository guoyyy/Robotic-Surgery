function d = distanceInserted(pneedle,rneedle,blockCorners)
%% TODO: barycentric coordinates of triangles should be passed to this function as parameters
EPSILON = 1e-6;
%%

%the numbered vertices of the block that define the corners of the faces
faces = [2,4,6,8;
    5,6,7,8;
    3,4,7,8;
    1,2,3,4;
    1,2,5,6;
    1,3,5,7];

global ln %assume a perfectly straight needle

%get a second needle point
pneedle2 = pneedle - (ln*rneedle)';
% pneedle2 = pneedle - ((ln*rneedle)/norm(ln*rneedle))';


intersectionCount = 0;
intersectionPoints = zeros(2,3);

%exit if the needletip is not in the block's bounding box
% if pneedle(1) < min(blockCorners(:,1))
%     d=0;
%     return;
% end
% if pneedle(1) > max(blockCorners(:,1))
%     d=0;
%     return;
% end
% if pneedle(2) < min(blockCorners(:,2))
%     d=0;
%     return;
% end
% if pneedle(2) > max(blockCorners(:,2))
%     d=0;
%     return;
% end
if pneedle(3) < min(blockCorners(:,3))
    d=0;
    return;
end
% if pneedle(3) > max(blockCorners(:,3))
%     d=0;
%     return;
% end


for i=1:6
    %new algorithm - segment/triangle intersection
    
    %TODO: this should only be calculated once, and not in this function.
    
    
    %get the coordinates of the face's corners
    faceCorners = blockCorners(faces(i,:),:);
    %split the face into triangles
    planecorners = [1 2 3;4 2 3];
    for j = 1:2
        %get vertices of current triangle
        verts = planecorners(j,:);
        %get coordinates of current triangle's vertices
        v0 = faceCorners(verts(1)',:);
        v1 = faceCorners(verts(2)',:);
        v2 = faceCorners(verts(3)',:);
                
        %get line segment
        P0 = pneedle2;
        D = rneedle;
        
        %carry out Moller & Trumbore's fast ray-triangle intersection algorithm
        edge1 = (v1-v0)';
        edge2 = (v2-v0)';
        
%                 pvec = cross(D,edge2);
        pvec = [D(2,:).*edge2(3,:)-D(3,:).*edge2(2,:)
            D(3,:).*edge2(1,:)-D(1,:).*edge2(3,:)
            D(1,:).*edge2(2,:)-D(2,:).*edge2(1,:)];
        
        det = edge1'*pvec; %dot product
        
        
%         %try the culling branch
%         if ~(det < EPSILON)
%             tvec = (P0-v0)';
%             u = tvec'*pvec;
%             if ~(u<0 || u > det)
%                 qvec = cross(tvec,edge1);
%                 v = D'*qvec;
%                 if ~(v < 0 || u+v>det)
%                     t = edge2'*qvec ;
%                     inv_det = 1/det;
%                     t = t*inv_det;
%                     u = t*inv_det;
%                     v = t*inv_det;
%                     if t <= ln
%                         % if t > ln, the needle is pointing toward the plane, but is not intersecting it
%                         intersectionCount = intersectionCount+1;
%                         
%                         % get the coordinates of the intersection point
%                         I = (1-u-v) * v0 + u*v1 + v*v2;
%                         intersectionPoints(intersectionCount,:) = I;
%                         if intersectionCount > 1
%                             break
%                         end
%                     end
%                 end
%             end  
%         end
        
        
        %note: i use the non-culling branch of the MT algorithm
        if ~(det > -EPSILON && det < EPSILON)
            %             d=0;
            %             break
            
            
            inv_det = 1/det;
            
            tvec = (P0-v0)';
            
            u = tvec'*pvec * inv_det;
            
            if ~(u < 0 || u > 1)
                
%                                 qvec = cross(tvec,edge1);
                qvec = [tvec(2,:).*edge1(3,:)-tvec(3,:).*edge1(2,:)
                    tvec(3,:).*edge1(1,:)-tvec(1,:).*edge1(3,:)
                    tvec(1,:).*edge1(2,:)-tvec(2,:).*edge1(1,:)];
                
                v = D' * qvec * inv_det;
                
                if ~(v < 0 || u+v > 1)
                    
                    t = edge2'*qvec * inv_det;
                    if t <= ln
                        % if t > ln, the needle is pointing toward the plane, but is not intersecting it
                        intersectionCount = intersectionCount+1;
                        
                        % get the coordinates of the intersection point
                        intersectionPoints(intersectionCount,:) = (1-u-v) * v0 + u*v1 + v*v2;
                        if intersectionCount > 1
                            break
                        end
                    end
                end
            end
        end
    end
end

if intersectionCount == 1
    intersectionPoints(2,:) = pneedle';
end

d = norm(intersectionPoints(1,:) - intersectionPoints(2,:));