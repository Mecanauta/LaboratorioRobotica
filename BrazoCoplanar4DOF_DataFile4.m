% Simscape(TM) Multibody(TM) version: 24.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(9).translation = [0.0 0.0 0.0];
smiData.RigidTransform(9).angle = 0.0;
smiData.RigidTransform(9).axis = [0.0 0.0 0.0];
smiData.RigidTransform(9).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [64 0 0];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[eslabon final para montaje simple-1:-:eslabon final para montaje simple-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [63.999999999999972 149.97634602523198 -0.02169550279036514];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918962595 -0.57735026918962562 0.57735026918962562];
smiData.RigidTransform(2).ID = "F[eslabon final para montaje simple-1:-:eslabon final para montaje simple-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 149.97634602523215 -0.021695502790436416];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[eslabon final para montaje simple-1:-:efector final-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [61.999999999999979 -7.5035494189643378e-14 2.9842794901924208e-13];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(4).ID = "F[eslabon final para montaje simple-1:-:efector final-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [82.999999999999957 58.000000000000021 -1.1102230246251565e-13];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(5).ID = "B[Base de eslabon 1-1:-:eslabon final para montaje simple-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-49.999999999999943 1.2434497875801753e-13 -1.4210854715202004e-14];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [0.57735026918962573 0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(6).ID = "F[Base de eslabon 1-1:-:eslabon final para montaje simple-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [0 64 0];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(7).ID = "B[Base-1:-:Base de eslabon 1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-5.5981278650929638e-15 -29.000000000000014 1.1354892353319396e-13];  % mm
smiData.RigidTransform(8).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(8).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(8).ID = "F[Base-1:-:Base de eslabon 1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-816.05264546350611 10.000000000000002 -192.78137817495426];  % mm
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = "RootGround[Base-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.54981161118720945;  % kg
smiData.Solid(1).CoM = [-16.402894705126357 16.863769202166939 0];  % mm
smiData.Solid(1).MoI = [728.35778563381484 1801.6819179297891 1611.6437743916242];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 242.77183160662534];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Base de eslabon 1*:*Predeterminado";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.14662080699800084;  % kg
smiData.Solid(2).CoM = [45.90185870597584 75.231742230102526 0.089212871719173012];  % mm
smiData.Solid(2).MoI = [516.69816098982699 203.49167311977763 695.94679419971828];  % kg*mm^2
smiData.Solid(2).PoI = [0.48950744813276897 0.20034734491780631 -129.76160111324782];  % kg*mm^2
smiData.Solid(2).color = [0.36470588235294116 1 0.62745098039215685];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "eslabon final para montaje simple*:*Predeterminado";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.052601184195396625;  % kg
smiData.Solid(3).CoM = [31.000000219141747 1.8264443968904629e-05 -21.464973273251452];  % mm
smiData.Solid(3).MoI = [38.125240653910353 54.491428179803471 22.055546324532813];  % kg*mm^2
smiData.Solid(3).PoI = [-1.3532166246128951e-05 -2.0003750261037073e-07 1.4753518587627672e-05];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "efector final*:*Predeterminado";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.78897646658255915;  % kg
smiData.Solid(4).CoM = [-0.65827519946425617 30.443790582610443 -0.6231757050250416];  % mm
smiData.Solid(4).MoI = [1396.1988657403956 1525.6036199152056 1394.7317055163205];  % kg*mm^2
smiData.Solid(4).PoI = [-6.6099214999096141 12.970646165012072 -6.9822160246461937];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Base*:*Predeterminado";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(4).Rz.Pos = 0.0;
smiData.RevoluteJoint(4).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 1.1017764609091714e-14;  % deg
smiData.RevoluteJoint(1).ID = "[eslabon final para montaje simple-1:-:eslabon final para montaje simple-2]";

smiData.RevoluteJoint(2).Rz.Pos = 89.999999999999986;  % deg
smiData.RevoluteJoint(2).ID = "[eslabon final para montaje simple-1:-:efector final-1]";

smiData.RevoluteJoint(3).Rz.Pos = 89.999999999999972;  % deg
smiData.RevoluteJoint(3).ID = "[Base de eslabon 1-1:-:eslabon final para montaje simple-2]";

smiData.RevoluteJoint(4).Rz.Pos = 87.177519666011392;  % deg
smiData.RevoluteJoint(4).ID = "[Base-1:-:Base de eslabon 1-1]";

