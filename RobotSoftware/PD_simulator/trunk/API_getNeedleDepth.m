function depth=API_getNeedleDepth(pneedle,rneedle)
rneedle=rneedle';
pneedle=pneedle';
blockCorners = [0 -240 360;
    0 -180 360; 
    0 -240 340; 
    0 -180 340; 
    -20 -240 360; 
    -20 -180 360; 
    -20 -240 340; 
    -20 -180 340];
blockCorners = reshape(blockCorners,8,3);
%rneedle = pexit-pentry;
rneedle = rneedle/norm(rneedle);
EPSILON = 1e-6;
%%

%the numbered vertices of the block that define the corners of the faces
faces = [2,4,6,8;
    5,6,7,8;
    3,4,7,8;%asdfasdf
    1,2,3,4;
    1,2,5,6;
    1,3,5,7];

ln= 68.6308+65.19+161.86+18.00+14.91; % (26 July 2010)
ln1 = 123.142;
ln2=ln-ln1;

%get a second needle point
pneedle2 = pneedle - ln*rneedle;

intersectionCount = 0;
intersectionPoints = zeros(2,3);

for i=1:6
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
        P0 = pneedle2';
        D = rneedle;
        
        %carry out Moller & Trumbore's fast ray-triangle intersection algorithm
        edge1 = (v1-v0)';
        edge2 = (v2-v0)';
        
        %pvec = cross(D,edge2);
        pvec = [D(2,:).*edge2(3,:)-D(3,:).*edge2(2,:)
            D(3,:).*edge2(1,:)-D(1,:).*edge2(3,:)
            D(1,:).*edge2(2,:)-D(2,:).*edge2(1,:)];
        
        det = edge1'*pvec; %dot product
        
        %note: i use the non-culling branch of the MT algorithm
        if ~(det > -EPSILON && det < EPSILON)
            
            inv_det = 1/det;
            tvec = (P0-v0)';
            u = tvec'*pvec * inv_det;
            
            if ~(u < 0 || u > 1)
                %qvec = cross(tvec,edge1);
                qvec = [tvec(2,:).*edge1(3,:)-tvec(3,:).*edge1(2,:)
                    tvec(3,:).*edge1(1,:)-tvec(1,:).*edge1(3,:)
                    tvec(1,:).*edge1(2,:)-tvec(2,:).*edge1(1,:)];
                
                v = D' * qvec * inv_det;
                
                if ~(v < 0 || u+v > 1)
                    
                    t = edge2'*qvec * inv_det;
                    if t <= ln
                        % if t > ln, the needle is pointing toward the plane, but is not intersecting it
                        %                         intersectionCount = intersectionCount+1;
                        %                         % get the coordinates of the intersection point
                        %                         intersectionPoints(intersectionCount,:) = (1-u-v) * v0 + u*v1 + v*v2;
                        %                         if intersectionCount > 1
                        %                             break
                        %                         end
                        %
                        
                        % get the coordinates of the intersection point
                        newintersection = (1-u-v) * v0 + u*v1 + v*v2;
                        
                        %add the new intersection, unless it's the case that
                        %we're splitting the triangles and the new point is the
                        %same as the first one
                        if intersectionCount == 0 || ~ isequal(newintersection, intersectionPoints(intersectionCount,:))
                            intersectionCount = intersectionCount+1;
                            intersectionPoints(intersectionCount,:) = newintersection;
                        end
                        
                        if intersectionCount >= 2
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


%now, for each point along the needle's path, find the distance between it
%and the intersection points

p0 = pneedle;
p1 = intersectionPoints(1,:)';
p2 = intersectionPoints(2,:)';

%dmax = norm(pentry-pexit);

% d1 = norm(p0 - p1);
% d2 = norm(p0 - p2);

depth = norm(p1-p2);


