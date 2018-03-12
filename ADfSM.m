%
%  ADfSM.m
%  ADfSM: All-around Depth from Small Motion with A Spherical Panoramic Cameras
% 
%
%  Created by Sunghoon Im on 2017. 1. 23..
%  Copyright @ 2017 Sunghoon Im. All rights reserved.
%


classdef ADfSM < handle
    
    properties
        dataset_root;
        dataset_name;
        video_format;
        max_iter;
        scaling;
        num_label;
        sam_rate;
    end
    
    methods
        
        function this = ADfSM(setting)
            this.dataset_root = setting.dataset_root;
            this.dataset_name = setting.dataset_name;
            this.video_format = setting.video_format;
            this.max_iter = setting.max_iter;
            this.scaling = setting.scaling;
            this.num_label = setting.num_label;
            this.sam_rate = setting.sam_rate;
        end
        
        
        function SMBA360(this)
        %% Bundle adjustment
            
            mkdir(fullfile(this.dataset_root,this.dataset_name));
            mkdir(fullfile(this.dataset_root,this.dataset_name,'L'));
            mkdir(fullfile(this.dataset_root,this.dataset_name,'R'));
            
            % VIdeo Read
            videoObj = VideoReader(fullfile(this.dataset_root,[this.dataset_name, '.', this.video_format]));
            num_frame=videoObj.NumberOfFrames;
            num_img = min(num_frame,30);
            
            ImgL=cell(num_img,1);
            ImgR=cell(num_img,1);
            
            sampling_rate=min(max(floor(num_frame/num_img),1),this.sam_rate);

            for i=1:num_img
                img=read(videoObj, 1+(i-1)*sampling_rate);
                ImgL{i} = img(:,1:end/2,:);
                ImgR{i} = img(:,end/2+1:end,:);
                ImgLsave = imresize(ImgL{i},this.scaling);
                ImgRsave = imresize(ImgR{i},this.scaling);
                imwrite(ImgLsave,fullfile(this.dataset_root,this.dataset_name,'L',sprintf('%04d.bmp',i-1)));
                imwrite(ImgRsave,fullfile(this.dataset_root,this.dataset_name,'R',sprintf('%04d.bmp',i-1)));
            end

            % Load camera parameters
            load(fullfile(this.dataset_root, 'RicohCalib.mat'));
            RT(1:3,4)=RT(1:3,4)/1000; % mm -> m
            RTLR = RT(1:3,:);
            RTinv = inv(RT);
            RTRL= RTinv(1:3,:);

            % Harris corner detection
            ImgrefL = rgb2gray(ImgL{1});
            [~,r,c] = harris(ImgrefL,1,1,1,0);
            featsL = [c,r];
            NfL = size(featsL, 1);

            ImgrefR = rgb2gray(ImgR{1});
            [~,r,c] = harris(ImgrefR,1,1,1,0);
            featsR = [c,r];
            NfR = size(featsR, 1);
            fprintf('Harris : %d and %d number of features are extracted\n', NfL, NfR)

            % KLT tracker
            tracker = vision.PointTracker('MaxBidirectionalError', 0.1);
            initialize(tracker, featsL, im2double(ImgL{1}));
            valid = ones(NfL, 1);
            for i = 2:num_img
               ImgT = im2double(ImgL{i});
               [feats_i, valid_i] = step(tracker, ImgT);
               featsL(:, 1+(i-1)*2 : 2+(i-1)*2) = feats_i;
               valid = valid + valid_i;
            end

            falsevalid = valid < num_img;
            featsL(falsevalid, :) = [];

            tracker = vision.PointTracker('MaxBidirectionalError', 0.1);
            initialize(tracker, featsR, im2double(ImgR{1}));
            valid = ones(NfR, 1);
            for i = 2:num_img
               ImgT = im2double(ImgR{i});
               [feats_i, valid_i] = step(tracker, ImgT);
               featsR(:, 1+(i-1)*2 : 2+(i-1)*2) = feats_i;
               valid = valid + valid_i;
            end

            falsevalid = valid < num_img;
            featsR(falsevalid, :) = [];

            NfL = size(featsL, 1);
            NfR = size(featsR, 1);
            fprintf('Tracking : %d and %d number of features are remaind\n', NfL, NfR);

            % Filtering by distance from center of image
            distL=sqrt((featsL(:,1:2:end)-KL(1,3)).^2 + (featsL(:,2:2:end)-KL(2,3)).^2);
            distL=max(distL,[],2);
            featsL(distL>430,:)=[];
            distR=sqrt((featsR(:,1:2:end)-KR(1,3)).^2 + (featsR(:,2:2:end)-KR(2,3)).^2);
            distR=max(distR,[],2);
            featsR(distR>430,:)=[];
            NfL = size(featsL, 1);
            NfR = size(featsR, 1);
            fprintf('Filtering : %d and %d number of features are remaind\n', NfL, NfR);

            % Feature color 
            color_featsL = zeros(3,NfL);
            color_featsL(1,:) = interp2(double(ImgL{1}(:,:,1)),featsL(:,1),featsL(:,2));
            color_featsL(2,:) = interp2(double(ImgL{1}(:,:,2)),featsL(:,1),featsL(:,2));
            color_featsL(3,:) = interp2(double(ImgL{1}(:,:,3)),featsL(:,1),featsL(:,2));

            color_featsR = zeros(3,NfR);
            color_featsR(1,:) = interp2(double(ImgR{1}(:,:,1)),featsR(:,1),featsR(:,2));
            color_featsR(2,:) = interp2(double(ImgR{1}(:,:,2)),featsR(:,1),featsR(:,2));
            color_featsR(3,:) = interp2(double(ImgR{1}(:,:,3)),featsR(:,1),featsR(:,2));

            % Initialization for bundle adjustment
            KL_inv=inv(KL);
            uL=featsL(:,1:2:end); vL=featsL(:,2:2:end);
            uL0=KL_inv(1,1)*uL+KL_inv(1,2)*vL+KL_inv(1,3); vL0=KL_inv(2,2)*vL+KL_inv(2,3);
            scaleL=(xiL+sqrt(1+(1-xiL^2)*(uL0.^2+vL0.^2)))./(uL0.^2+vL0.^2+1);
            xL=scaleL.*uL0; yL=scaleL.*vL0; zL=scaleL.*ones(size(xL))-xiL*ones(size(xL));
            XL=reshape([xL(:),yL(:),zL(:)]',[3,NfL,num_img]);
            wL=0.1*ones(NfL,1);

            KR_inv=inv(KR);
            uR=featsR(:,1:2:end); vR=featsR(:,2:2:end);
            uR0=KR_inv(1,1)*uR+KR_inv(1,2)*vR+KR_inv(1,3); vR0=KR_inv(2,2)*vR+KR_inv(2,3);
            scaleR=(xiR+sqrt(1+(1-xiR^2)*(uR0.^2+vR0.^2)))./(uR0.^2+vR0.^2+1);
            xR=scaleR.*uR0; yR=scaleR.*vR0; zR=scaleR.*ones(size(xR))-xiR*ones(size(xR));
            XR=reshape([xR(:),yR(:),zR(:)]',[3,NfR,num_img]);
            wR=0.1*ones(NfR,1);

            rt_init=zeros(6,num_img);
            
            
            % Bundle adjustment
            fprintf('Start bundle adjuetment\n');
            for iter=1:this.max_iter
                fo=fopen(fullfile(this.dataset_root,this.dataset_name,'smba.dat'),'wb');
                fwrite(fo,num_img,'int');
                fwrite(fo,NfL,'int');
                fwrite(fo,NfR,'int');
                fwrite(fo,RTLR','double');
                fwrite(fo,RTRL','double');
                fwrite(fo,rt_init,'double');
                fwrite(fo,wL,'double');
                fwrite(fo,wR,'double');
                fwrite(fo,XL,'double');
                fwrite(fo,XR,'double');
                fclose(fo);

                tic;
                system(sprintf('SMBA360.exe %s',fullfile(this.dataset_root,this.dataset_name)));
                elapsedTime=toc;
                fprintf(1,'Elapsed time is %.6f seconds.\n',elapsedTime);

                fi = fopen(fullfile(this.dataset_root,this.dataset_name, 'smba_result.dat'), 'rb');
                rt_BA=fread(fi,[6,num_img],'double');
                W_BAL=fread(fi,[1,NfL],'double');
                W_BAR=fread(fi,[1,NfR],'double');
                fclose(fi);

                mask_W_BAL=(W_BAL<0);
                mask_W_BAR=(W_BAR<0);

                r_BA=rt_BA(1:3,:); t_BA=rt_BA(4:6,:);
                X_BAL = XL(:,:,1)./repmat(W_BAL,3,1);
                X_BAR = XR(:,:,1)./repmat(W_BAR,3,1);

                D_BAL=sqrt(sum(X_BAL.^2));
                D_BAR=sqrt(sum(X_BAR.^2));
                thresh_D=prctile([D_BAL,D_BAR],98);
                mask_D_BAL=(D_BAL>thresh_D);
                mask_D_BAR=(D_BAR>thresh_D);

                FL=fn_reprojectionerror360(eye(4), rt_BA, eye(4), X_BAL, XL);
                FR=fn_reprojectionerror360(RTRL, rt_BA, RTLR, X_BAR, XR);

                SSEL=sum(sum(FL.^2,1),3);
                SSER=sum(sum(FR.^2,1),3);

                thresh_SSE=prctile([SSEL,SSER],95);
                mask_SSEL=(SSEL>=thresh_SSE);
                mask_SSER=(SSER>=thresh_SSE);

                outlierL=find(mask_W_BAL|mask_D_BAL|mask_SSEL);
                outlierR=find(mask_W_BAR|mask_D_BAR|mask_SSER);

                W_BAL(outlierL)=[];
                W_BAR(outlierR)=[];
                XL(:,outlierL,:)=[];
                XR(:,outlierR,:)=[];
                featsL(outlierL,:)=[];
                featsR(outlierR,:)=[];
                color_featsL(:,outlierL)=[];
                color_featsR(:,outlierR)=[];

                NfL=size(featsL,1);
                NfR=size(featsR,1);
                rt_init=rt_BA;
                wL=W_BAL;
                wR=W_BAR;
                X_BAL=XL(:,:,1)./repmat(W_BAL,3,1);
                X_BAR=XR(:,:,1)./repmat(W_BAR,3,1);
            end
            fprintf('Done SfSM\n')
            
            X_BARL = RTRL*[X_BAR; ones(1,NfR)];
            XCL=[X_BAL; color_featsL/255];
            XCR=[X_BAR; color_featsR/255];
            XCRL=[X_BARL; color_featsR/255];

            fprintf('Final : %d and %d number of features are extracted\n', length(XCL), length(XCR));

            % Generate 3D
            cmap=jet(num_img);

            CCL=zeros(6,num_img);
            CCR=zeros(6,num_img);
            RTL=zeros(4,4,num_img);
            RTR=zeros(4,4,num_img);

            for i=1:num_img
                Rtemp = fn_eulervec2mat(r_BA(1:3,i));
                Ttemp = t_BA(:,i);
                RTtempL = [Rtemp Ttemp; 0 0 0 1];
                RTtempR = [RTLR; 0 0 0 1] * RTtempL;
                RTL(:,:,i) = RTtempL;
                RTR(:,:,i) = [RTLR;0 0 0 1]*RTtempL*[RTRL; 0 0 0 1];
                ccL = -RTtempL(1:3,1:3)'*RTtempL(1:3,4);
                ccR = -RTtempR(1:3,1:3)'*RTtempR(1:3,4);
                CCL(:,i)=[ccL; cmap(i,:)'];
                CCR(:,i)=[ccR; cmap(i,:)'];
            end
            fn_saveply(fullfile(this.dataset_root,this.dataset_name,'SMBA360.ply'), [XCL, XCRL, CCL, CCR]);

            % Save data
            save(fullfile(this.dataset_root,this.dataset_name,'SMBA360.mat'),'XCL','XCR', 'RTL', 'RTR');
        end
         
        function SphereSweep360(this)
            %% Dense matching

            % reference image
            IrefL=im2uint8(imread(fullfile(this.dataset_root,this.dataset_name,'L',sprintf('%04d.bmp',0))));
            IrefR=im2uint8(imread(fullfile(this.dataset_root,this.dataset_name,'R',sprintf('%04d.bmp',0))));
            IrefLR=[IrefL,IrefR];
            [ROW, COL, ~] = size(IrefL);
            
            % load calib
            load(fullfile(this.dataset_root,  'RicohCalib.mat'));
            RT(1:3,4)=RT(1:3,4)/1000;
            RTLR = RT(1:3,:);
            RTinv = inv(RT);
            RTRL= RTinv(1:3,:);

            % load BA data
            load(fullfile(this.dataset_root,this.dataset_name,'SMBA360.mat'));
            num_img=size(RTL,3);

            depths=[ sqrt(XCL(1,:).^2+XCL(2,:).^2+XCL(3,:).^2), sqrt(XCR(1,:).^2+XCR(2,:).^2+XCR(3,:).^2) ];
            min_depth=min(depths)/2;
            max_depth=max(depths)*2;

            % initialize dense matching
            KL(1:2,:) = KL(1:2,:)*this.scaling;
            KR(1:2,:) = KR(1:2,:)*this.scaling;
            kvecL=[KL(1,1),KL(2,2),KL(1,2),KL(1,3),KL(2,3)];
            kvecR=[KR(1,1),KR(2,2),KR(1,2),KR(1,3),KR(2,3)];
            RTLvec=reshape(RTL(1:3,:,:),[12,num_img]);
            RTRvec=reshape(RTR(1:3,:,:),[12,num_img]);

            min_angle=98;
            max_angle=100;
            lambda=10;

            w0=1/max_depth;
            dw=(1/min_depth-w0)/(this.num_label-1);

            fo=fopen(fullfile(this.dataset_root,this.dataset_name,'sweep.dat'),'wb');
            fwrite(fo,num_img,'int');
            fwrite(fo,this.num_label,'int');
            fwrite(fo,min_angle,'float');
            fwrite(fo,max_angle,'float');
            fwrite(fo,lambda,'float');
            fwrite(fo,w0,'float');
            fwrite(fo,dw,'float');
            fwrite(fo,kvecL,'float');
            fwrite(fo,kvecR,'float');
            fwrite(fo,xiL,'float');
            fwrite(fo,xiR,'float');
            fwrite(fo,RTLR','float');
            fwrite(fo,RTRL','float');
            fwrite(fo,RTLvec,'float');
            fwrite(fo,RTRvec,'float');
            fclose(fo);

            fprintf('Start Dense Matching\n')
            tic;
            system(sprintf('SphereSweep360.exe %s bmp',fullfile(this.dataset_root,this.dataset_name)));
            elapsedTime=toc;
            fprintf(1,'Elapsed time is %.6f seconds.\n',elapsedTime);
            fprintf('Done Dense Matching\n')

            %% Depth refinement
            % Left image
            fi=fopen(fullfile(this.dataset_root,this.dataset_name,'costvolumeL.dat'),'rb');
            costvolumeL=fread(fi,'int');
            fclose(fi);
            costvolumeL=reshape(costvolumeL,[COL,ROW,this.num_label]);
            costvolumeL = permute(costvolumeL,[2,1,3]);
            [mincostL, labelL] = min(costvolumeL,[],3);
            medianL = median(costvolumeL,3);
            confidenceL = mincostL./medianL;
            threshL=prctile(confidenceL(:),80);
            labelL_tree_hole=labelL-1;
            labelL_tree_hole(confidenceL>threshL)=0;

            % Right image
            fi=fopen(fullfile(this.dataset_root,this.dataset_name,'costvolumeR.dat'),'rb');
            costvolumeR=fread(fi,'int');
            fclose(fi);
            costvolumeR=reshape(costvolumeR,[COL,ROW,this.num_label]);
            costvolumeR = permute(costvolumeR,[2,1,3]);
            [mincostR, labelR] = min(costvolumeR,[],3);
            medianR = median(costvolumeR,3);
            confidenceR = mincostR./medianR;
            threshR=prctile(confidenceR(:),80);
            labelR_tree_hole=labelR-1;
            labelR_tree_hole(confidenceR>threshR)=0;
            
            labelLR=[labelL,labelR];
            labelLR_tree_hole=[labelL_tree_hole,labelR_tree_hole];
            labelLR_tree=tree_upsampling(IrefLR,uint8(labelLR_tree_hole),256,0.01,1);
            
            % save
            imwrite(uint8(labelLR*255/this.num_label),fullfile(this.dataset_root,this.dataset_name, 'depth_LR.bmp'));
            imwrite(uint8(labelLR_tree_hole*255/this.num_label),fullfile(this.dataset_root,this.dataset_name, 'depth_LR_hole.bmp'));
            imwrite(uint8(labelLR_tree*255/this.num_label),fullfile(this.dataset_root,this.dataset_name, 'depth_LR_tree.bmp'));
            
            depthLR=w0+dw*labelLR_tree;
            wL = 1./depthLR(:,1:COL);
            wR = 1./depthLR(:,COL+1:end);
            
            rth = 230;
            [u,v]=meshgrid(1:COL,1:ROW);
            u=u(:); v=v(:); 
            rL=sqrt((u-KL(1,3)).^2 + (v-KL(2,3)).^2);
            rR=sqrt((u-KR(1,3)).^2 + (v-KR(2,3)).^2);
            maskL=reshape(rL>rth,[ROW,COL]);
            maskR=reshape(rR>rth,[ROW,COL]);
            
            save(fullfile(this.dataset_root,this.dataset_name,'SS360.mat'),'IrefL','IrefR','KL','KR','RTLR','RTRL','xiL','xiR','maskL','maskR','wL','wR');
            
        end
        
        function Sphericaldepth(this)
            load(fullfile(this.dataset_root,this.dataset_name,'SS360.mat'));
          
            %% Get 3D points
                % get the 3D left
            [ROWL,COLL,~]=size(IrefL);
            [xL, yL] = meshgrid(1:COLL,1:ROWL);
            xL(maskL==1) = KL(1,3);
            yL(maskL==1) = KL(2,3);

            XYZL = Im2Sph(xL,yL,KL,eye(3),xiL); 
            Pts3DL = XYZL./repmat(wL(:),[1,3]); Pts3DL(:,4) = 1;

                % get the 3D right
            [ROWR,COLR,~]=size(IrefR);
            [xR, yR] = meshgrid(1:COLR,1:ROWR);
            xR(maskR==1) = KR(1,3);
            yR(maskR==1) = KR(2,3);

            XYZR = Im2Sph(xR,yR,KR,RTLR(1:3,1:3),xiR); 
            Pts3DR = XYZR./repmat(wR(:),[1,3]); Pts3DR(:,4) = 1;
            
            %% Create stereo panorama
                % depth
            Rs = getRotationMat(0,270,0); Ts = [0 0 0]';
            wLL = repmat(1./wL,[1,1,3]); wRR = repmat(1./wR,[1,1,3]);
            DTL = SyntheticPano(wLL,wRR,Pts3DL,Pts3DR,1920,Rs,Ts);
            DTL2 = DTL/max(DTL(:));
            
                % image
            Rs = getRotationMat(0,270,0); Ts = [0 0 0]';
            ImTL = SyntheticPano(IrefL,IrefR,Pts3DL,Pts3DR,1920,Rs,Ts);
            imwrite(ImTL,fullfile(this.dataset_root,this.dataset_name, 'panoimg.bmp'));
            imwrite(DTL2,fullfile(this.dataset_root,this.dataset_name, 'panodepth.bmp'));
            
            save(fullfile(this.dataset_root,this.dataset_name,'Synpano.mat'),'DTL','ImTL');
        end
    end
end


function R=fn_eulervec2mat(r)
    s1=sin(r(1)); c1=cos(r(1));
    s2=sin(r(2)); c2=cos(r(2));
    s3=sin(r(3)); c3=cos(r(3));
    
    R=[ c2*c3,      s1*s2*c3-c1*s3,     c1*s2*c3+s1*s3;
        c2*s3,      s1*s2*s3+c1*c3,     c1*s2*s3-s1*c3;
        -s2,        s1*c2,              c1*c2;];
end

function F=fn_reprojectionerror360(tf1, rt, tf2, X, x)

    n=size(rt,2);
    nf=size(X,2);
    
    F=zeros(3,nf,n);
    
    X0=X;
    X0(4,:)=1;
    tf1(4,:)=[0,0,0,1];
    tf2(4,:)=[0,0,0,1];

    for i=1:n
        R=fn_eulervec2mat(rt(1:3,i));
        tf=[ R, rt(4:6,i);
            0, 0, 0, 1];
        
        X1=tf1*X0;
        X2=tf*X1;
        X3=tf2*X2;
        
        scale=sqrt(sum(X3(1:3,:).^2));
        X4=X3(1:3,:)./repmat(scale,3,1);
        xi=x(:,:,i);
        F(:,:,i)= X4-xi;
    end
end
function XYZ= Im2Sph(x,y,K,R,xi)

    %frame conversion
    x_t = (x-K(1,3))/K(1,1);
    y_t = (y-K(2,3))/K(2,2);

    %Projection to the sphere
    z=x_t.^2+y_t.^2;

    im2s = (xi + sqrt(1+(1-xi.*xi).*z))./(z+1);
    Xs = im2s.*x_t;
    Ys = im2s.*y_t;
    Zs = im2s-xi;
    % 
    % Zs=((-2*xi*z)+sqrt((2*xi*z).^2-4*(z+1).*(xi^2*z-1)))./(2*(z+1));
    % Xs=x_t.*(Zs+xi);
    % Ys=y_t.*(Zs+xi);

    %Rotation
    XYZ=[Xs(:) Ys(:) Zs(:)]*R;
    XYZ = real(XYZ);
end

function ImT = SyntheticPano(IrefL,IrefR,Pts3DL,Pts3DR,COLpano,Rs,Ts)

    M1 = [Rs Ts; 0 0 0 1];

    [ROW, COL, CH]=size(IrefL);
    % full point cloud
    colorL = reshape(im2double(IrefL), [ROW*COL,3]);
    colorR = reshape(im2double(IrefR), [ROW*COL,3]);
    color = [colorL; colorR];

    Pts3D = [Pts3DL; Pts3DR];
    % project on the sphere
    PT1 = M1*Pts3D';
    norm1 = sqrt(sum(PT1(1:3,:).^2));
    in=(norm1>eps);
    XYZ = PT1(1:3,:)./repmat(norm1,[3,1]);

    ROWpano = COLpano/2;
    [the, phi, ~]=cart2sph(XYZ(1,:),XYZ(2,:),XYZ(3,:));
    the = (the+pi)/(2*pi); phi = (phi+pi/2)/(pi);
    coorpano = round([the*(COLpano-1)+1; phi*(ROWpano-1)+1]);

    [cc, rr] = meshgrid(1:COLpano, 1:ROWpano);

    ImT = zeros(ROWpano, COLpano, CH);
    for ch = 1:CH
        imt = griddata(coorpano(1,in),coorpano(2,in), color(in,ch)', cc(:), rr(:), 'cubic');
        ImT(:,:,ch) = reshape(imt, [ROWpano, COLpano]);
    end
end

function DisplaySphere(ImFL, ImFR, KL, KR, xiL, xiR, RL, RR, maskL, maskR)

    % Display sphere 1
    [ROWL,COLL,~]=size(ImFL);
    [x1, y1] = meshgrid(1:COLL, 1:ROWL);
    x1(maskL==1) = KL(1,3);
    y1(maskL==1) = KL(2,3);

    XYZL = Im2Sph(x1,y1,KL,RL,xiL);
    XsfL = reshape(XYZL(:,1),[ROWL,COLL]); YsfL = reshape(XYZL(:,2),[ROWL,COLL]); ZsfL = reshape(XYZL(:,3),[ROWL,COLL]);

    % Display sphere 2
    [ROWR,COLR,~]=size(ImFR);
    [x2, y2] = meshgrid(1:COLR, 1:ROWR);
    x2(maskR==1) = KR(1,3);
    y2(maskR==1) = KR(2,3);

    XYZR = Im2Sph(x2,y2,KR,RR,xiR);
    XsfR = reshape(XYZR(:,1),[ROWR,COLR]); YsfR = reshape(XYZR(:,2),[ROWR,COLR]); ZsfR = reshape(XYZR(:,3),[ROWR,COLR]);



    % Sphere display configuration
    k = 1; n = 2^k-1; %Face on the sphere
    [x,y,z] = sphere(n); c = linspace(-2,-2,n); c=repmat(c,n,1); %Generation of the sphere

    figure; hold on;
    surf(XsfL,YsfL,ZsfL,mat2gray(ImFL));
    surf(XsfR,YsfR,ZsfR,mat2gray(ImFR));
    shading interp;
    surface(x,y,z,'EdgeColor','none','FaceColor',[0.8 0.8 0.8])  % display sphere
    axis equal;
    hold off;
end

function [rot_mat] = getRotationMat(roll,pitch,yaw)

    %roll, pitch, yaw expressed in degree
    % x, y, z

    Rp=[cos(deg2rad(pitch)) 0 sin(deg2rad(pitch));...
        0 1 0;...
        -sin(deg2rad(pitch)) 0 cos(deg2rad(pitch))]; %Rot_y

    Ry=[cos(deg2rad(yaw)) -sin(deg2rad(yaw)) 0;...
        sin(deg2rad(yaw)) cos(deg2rad(yaw)) 0;...
        0 0 1]; %Rot_z

    Rr=[1 0 0;...
        0 cos(deg2rad(roll)) -sin(deg2rad(roll));...
        0 sin(deg2rad(roll)) cos(deg2rad(roll))]; %Rot_x

    rot_mat = Ry*Rp*Rr;
    % rot_mat = Rr*Rp*Ry;

end

function fn_saveply(filename, X)
    out=fopen(filename,'w');
    fprintf(out,'ply\n');
    fprintf(out,'format ascii 1.0\n');
    fprintf(out,'element vertex %d\n',size(X,2));
    fprintf(out,'property float x\n');
    fprintf(out,'property float y\n');
    fprintf(out,'property float z\n');
    fprintf(out,'property uchar diffuse_red\n');
    fprintf(out,'property uchar diffuse_green\n');
    fprintf(out,'property uchar diffuse_blue\n');
    fprintf(out,'end_header\n');
    for i=1:size(X,2)
        fprintf(out,'%f %f %f %d %d %d\n',[X(1,i),X(2,i),X(3,i),min(round(X(4,i)*255),255),min(round(X(5,i)*255),255),min(round(X(6,i)*255),255)]);
    end
    fclose(out);
end
