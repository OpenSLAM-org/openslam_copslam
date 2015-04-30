%%
%%
%%
%% This function can be used to visualize COP-SLAM's G2O files in matlab/octave
%% Author: Gijs Dubbelman (gijsdubbelman@gmail.com)
%%
%%
function [] = showG2OFiles()

    %% check if we are using octacve ot matlab
    isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
    if isOctave
      graphics_toolkit('fltk')
    end    

    %% parse the file
    [track1 loops1]  = parseG2OFile( '../data/KITTI_00_cs.g2o' );
    [track2 loops2]  = parseG2OFile( '../data/KITTI_02_cs.g2o' );
    [track3 loops3]  = parseG2OFile( '../data/NewCollege_cs.g2o' );
    [track4 loops4]  = parseG2OFile( '../data/Pittsburgh_A_cs.g2o' );
    [track5 loops5]  = parseG2OFile( '../data/Pittsburgh_B_cs.g2o' );
    [track6 loops6]  = parseG2OFile( '../data/Pittsburgh_C_cs.g2o' );
    [track7 loops7]  = parseG2OFile( '../data/TheHague_02_cs.g2o' );
    [track8 loops8]  = parseG2OFile( '../data/sphere_cs.g2o' );
    
    %% show the trajectory    
    figure(1)
    clf
    hold on
    title('KITTI 00');
    plotTrack( track1, loops1 );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')

    %% show the trajectory    
    figure(2)
    clf
    hold on
    title('KITTI 02');
    plotTrack( track2, loops2  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(3)
    clf
    hold on
    title('New College');
    plotTrack( track3, loops3  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(4)
    clf
    hold on
    title('Pittsburgh A');
    plotTrack( track4, loops4  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(5)
    clf
    hold on
    title('Pittsburgh B');
    plotTrack( track5, loops5  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(6)
    clf
    hold on
    title('Pittsburgh C');
    plotTrack( track6, loops6  );
    view(-50,30);
    grid on
    axis equal    
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(7)
    clf
    hold on
    title('The Hague 2');
    plotTrack( track7, loops7  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
    %% show the trajectory    
    figure(8)
    clf
    hold on
    title('The Sphere');
    plotTrack( track8, loops8  );
    view(-50,30);
    grid on
    axis equal
    legend('trajectory','loop closures','Location','NorthWest')
    
return



%%
%% Parse absolute positions from a g2o file
%%
function [track, loops] = parseG2OFile( filename )
    
    % open the file
    fid   = fopen(filename);
    verts = fscanf(fid,'VERTEX_SE3:QUAT %i %f %f %f %f %f %f %f\n' ,[inf]);
    loops = fscanf(fid,'EDGE_SE3:QUAT %i %i %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' ,[inf]);
    if isempty( verts )
        fseek(fid, 0, 'bof');
        verts  = fscanf(fid,'VERTEX_RST3:QUAT %i %f %f %f %f %f %f %f %f\n' ,[inf]);
        loops  = fscanf(fid,'EDGE_RST3:QUAT %i %i %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' ,[inf]);
        verts  = reshape(verts,9,size(verts,1)/9);
        verts  = verts(1:8,:);
        loops  = reshape(loops,31,size(loops,1)/31);
        loops  = loops(1:10,:);        
    else
        verts  = reshape(verts,8,size(verts,1)/8);
        loops  = reshape(loops,30,size(loops,1)/30);
        loops  = loops(1:2,:);
    end    
    track = verts(2:4,:);
    idx   = find( 1 < abs(loops(1,:) - loops(2,:)) );
    loops = loops(:,idx);
    fclose( fid );
    
return



%%
%% Plot trajectory in 3D
%%
function [] = plotTrack( track, loops )

    % plot in 3D   
    plot3( track(1,:), track(3,:), track(2,:) );
    
    % plot loop closures
    for n = 1:size(loops,2)
        x = [track(1,loops(1,n)+1) track(1,loops(2,n)+1)];
        y = [track(2,loops(1,n)+1) track(2,loops(2,n)+1)];
        z = [track(3,loops(1,n)+1) track(3,loops(2,n)+1)];
        plot3( x, z, y, '-or' );
    end
    

return
