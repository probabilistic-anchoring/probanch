%%% -*- Mode: Prolog; -*-
:- use_module(library(distributionalclause)).
:- use_module(library(dcpf)).
:- set_options(default).
:- set_inference(backward(lazy)).

:- use_module(library(lists)).

builtin(deltaT(_)).
builtin(varQ(_)).
builtin(cov(_,_,_)).

deltaT(0.2).
varQ(0.0001).

% http://webee.technion.ac.il/people/shimkin/Estimation09/ch8_target.pdf page 3
cov(2,[Cov11,Cov12,Cov21,Cov22],VarQ) :-
	deltaT(DeltaT),
	Cov11 is VarQ*(DeltaT^3)/3,
	Cov12 is VarQ*(DeltaT^2)/2,
	Cov21 is VarQ*(DeltaT^2)/2,
	Cov22 is VarQ*(DeltaT).

cov(1,[Cov,0,0,0.000001],VarQ) :-
	deltaT(DeltaT),
	Cov is VarQ*DeltaT^2.

time(0):t+1 <-
	\+time(_):t.
time(T1):t+ 1<-
	time(T0):t,
	T1 is T0+1.


remove_anchor_list:t+1 ~ val(RA_List) <-
   findall_forward(RA_ID, (observation(remove_anchor(RA_ID))~=_), RA_List).
removed(RA_ID):t+1 <-
   remove_anchor_list:t+1 ~= RA_List,
   member(RA_ID,RA_List).

observed_list:t+1 ~ val(A_List) <-
   findall_forward(A_ID, (observation(anchor_r(A_ID))~=_), A_List).
observed(A_ID):t+1 <-
   observed_list:t+1 ~= A_List,
   member(A_ID,A_List).


anchor(A_ID):t+1 <-
	observed(A_ID):t+1,
   \+removed(A_ID):t+1.
anchor(A_ID):t+1 <-
	anchor(A_ID):t,
   \+removed(A_ID):t+1,
   occluded_by(A_ID,_):t+1.





dotProd([H1|T1], [H2|T2], Prod):t <-
   dotProd(T1, T2, PartProd):t,
   Prod is H1*H2 + PartProd.
dotProd([], [H2], Prod):t <-
   Prod is H2.
logistic(P,X,Result):t <-
   dotProd(X,P,Mean):t,
   Result is 1/(1 + exp(-1*Mean)).
distance(A_ID,A_ID_occluder,D):t <-
    rv(A_ID):t ~= (X1,_,Y1,_,Z1,_),
    rv(A_ID_occluder):t+1 ~= (X2,_,Y2,_,Z2,_),
    D is sqrt((X1-X2)^2+(Y1-Y2)^2+(Z1-Z2)^2).
distancexy(A_ID,A_ID_occluder,D):t <-
    rv(A_ID):t ~= (X1,_,Y1,_,_,_),
    rv(A_ID_occluder):t+1 ~= (X2,_,Y2,_,_,_),
    D is sqrt((X1-X2)^2+(Y1-Y2)^2).


occluded_by(A_ID,A_ID_occluder):t+1 <-
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   category(A_ID):t~=Cat,
   \+member(Cat, [glove, skin]),
   pick_occluder_transition(A_ID):t+1 ~= A_ID_occluder.
pick_occluder_transition(A_ID):t+1 ~ uniform(Occluders) <-
   anchor(A_ID):t,
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   findall_forward(
      A_ID_occluder,
      (
         observed(A_ID_occluder):t+1,
         occluded_by_possible_transition(A_ID, A_ID_occluder):t~=true,
         category(A_ID_occluder):t~=Cat,
         member(Cat, [box,mug])
      ),
      Occluders
   ),
   \+Occluders=[].

% Learned
% --------------------------
occluded_by_possible_transition(A_ID,A_ID_occluder):t ~ finite([1.0:false]) <-
   observed(A_ID):t+1.
occluded_by_possible_transition(A_ID,A_ID_occluder):t ~ finite([P1:false,P2:true]) <-
   distancexy(A_ID,A_ID_occluder,Distance):t,
   \+observed(A_ID):t+1,
   logistic([-16.48841732233956, 1.9484488282808254],[Distance],P2):t,
   P1 is 1.0-P2.
occluded_by_possible_transition(A_ID,A_ID_occluder):t ~ finite([1.0:false]) <-
   \+distancexy(A_ID,A_ID_occluder,_):t.
occluded_by_possible_transition(A_ID,A_ID_occluder):t ~ finite([0.7758620689655172:false,0.22413793103448276:true]) <-
   true.
% --------------------------





occluded_by(A_ID,A_ID_occluder):t+1 <-
   anchor(A_ID):t,
   occluded_by(A_ID,_):t,
   \+observed(A_ID):t,
   pick_occluder_tracking(A_ID):t+1 ~= A_ID_occluder.
pick_occluder_tracking(A_ID):t+1 ~ uniform(Occluders) <-
   findall_forward(
      A_ID_occluder,
      (
         observed(A_ID_occluder):t+1,
         occluded_by_possible_tracking(A_ID, A_ID_occluder):t~=true
      ),
      Occluders
   ),
   \+Occluders=[].
% Learned
% --------------------------
occluded_by_possible_tracking(A_ID,A_ID_occluder):t ~ finite([1.0:true]) <-
   occluded_by(A_ID,A_ID_occluder):t,
   \+observed(A_ID):t+1.
occluded_by_possible_tracking(A_ID,A_ID_occluder):t ~ finite([1.0:false]) <-
   \+occluded_by(A_ID,A_ID_occluder):t.
occluded_by_possible_tracking(A_ID,A_ID_occluder):t ~ finite([0.26949904882688647:true,0.7305009511731135:false]) <-
   true.
% --------------------------






% occluded_by(A_ID,A_ID_occluder):t+1 <-
%    anchor(A_ID):t,
%    anchor(A_ID_occluder):t,
%    occluded_by(A_ID,A_ID_occluder):t,
%    \+observed(A_ID):t+1,
%    observed(A_ID_occluder):t+1.

occluded_by(A_ID,A_ID_occluder):t+1 <-
   anchor(A_ID):t,
   anchor(A_ID_occluder):t,
   occluded_by(A_ID,A_ID_occluder):t,
   \+observed(A_ID):t+1,
   \+observed(A_ID_occluder):t+1,
   occluded_by(A_ID_occluder,_):t+1.



% observation position
observation(anchor_r(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation color
observation(anchor_c(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation category
observation(anchor_category(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation bb
observation(anchor_bb(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation remove
observation(remove_anchor(RA_ID)):t+1 ~ val(_) <-
   true.



%position transition
rv(A_ID):t+1 ~  indepGaussians([ ([X,0],Cov), ([Y,0],Cov), ([Z,0],Cov) ]) <-
	observation(anchor_r(A_ID)) ~= (X,Y,Z),
	\+anchor(A_ID):t,
	anchor(A_ID):t+1,
	varQ(VarQ),
	VarQ1 is VarQ/5,
	cov(1,Cov,VarQ1).
rv(A_ID):t+1 ~ indepGaussians([ ([O_x,0],Cov), ([O_y,0],Cov), ([O_z,0],Cov) ]) <-
	observation(anchor_r(A_ID)) ~= (O_x,O_y,O_z),
	anchor(A_ID):t,
	anchor(A_ID):t+1,
	rv(A_ID):t ~= (R_x,_,R_y,_,R_z,_),
	V_x is O_x-R_x,
	V_y is O_y-R_y,
	V_z is O_z-R_z,
	varQ(VarQ),
	VarQ1 is VarQ/3,
	cov(1,Cov,VarQ1).

noisy(_,_,Pos):t+1 ~ gaussian(Pos,0.00005) <- true.
rv(A_ID):t+1 ~ val((X,V_x,Y,V_y,Z,V_z)) <-
   anchor(A_ID):t,
   anchor(A_ID):t+1,
   occluded_by(A_ID,A_ID_Occluder):t+1,
	rv(A_ID_Occluder):t+1 ~= (R_x,V_x,R_y,V_y,R_z,V_z),
   noisy(A_ID,x,R_x):t+1 ~= X,
   noisy(A_ID,y,R_y):t+1 ~= Y,
   noisy(A_ID,z,R_z):t+1 ~= Z.






color(A_ID):t+1 ~ val(C) <-
	anchor(A_ID):t+1,
   \+anchor(A_ID):t,
	observation(anchor_c(A_ID)) ~= C.
color(A_ID):t+1 ~ val(C) <-
	anchor(A_ID):t+1,
	color(A_ID):t ~= C.

category(A_ID):t+1 ~ val(Category) <-
	anchor(A_ID):t+1,
	observation(anchor_category(A_ID)) ~= Category.
category(A_ID):t+1 ~ val(Caffe) <-
	anchor(A_ID):t+1,
	category(A_ID):t ~= Caffe.

bb(A_ID):t+1 ~ val(BB) <-
	anchor(A_ID):t+1,
	observation(anchor_bb(A_ID)) ~= BB.
bb(A_ID):t+1 ~ val(BB) <-
	anchor(A_ID):t+1,
	bb(A_ID):t ~= BB.
