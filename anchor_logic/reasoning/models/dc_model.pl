%%% -*- Mode: Prolog; -*-
:- use_module(library(distributionalclause)).
:- use_module(library(dcpf)).


:- use_module(library(lists)).
:- set_options(default).
:- set_inference(backward(lazy)).



% builtin(varia(_,_,_)).
% builtin(product(_,_)).
% builtin(densityGaussian(_,_,_,_)).
% builtin(normalize(_,_)).
% builtin(optimalproposal(_,_,_,_,_,_,_)).
% builtin(logIndepOptimalProposals(_,_,_)).

%
% checkvalue(true,1.0).
% checkvalue(false,0.0).


%
builtin(visionfield(_,_)).
builtin(rgbcolor(_,_)).
builtin(deltaT(_)).
builtin(varQ(_)).
builtin(cov(_,_,_)).

% builtin(append(_,_,_)).


% builtin(list_to_ord_set(_,_)).


deltaT(0.2).
varQ(0.01).

% http://webee.technion.ac.il/people/shimkin/Estimation09/ch8_target.pdf page 3
cov(2,[Cov11,Cov12,Cov21,Cov22],VarQ) :-
	deltaT(DeltaT),
	Cov11 is VarQ*(DeltaT^3)/3,
	Cov12 is VarQ*(DeltaT^2)/2,
	Cov21 is VarQ*(DeltaT^2)/2,
	Cov22 is VarQ*(DeltaT).

cov(1,[Cov,0,0,0.0000001],VarQ) :-
	deltaT(DeltaT),
	Cov is VarQ*DeltaT^2.


%percentage rgb
rgbcolor(white,(1.0,1.0,1.0)).
rgbcolor(gray,(0.3,0.3,0.3)).
rgbcolor(black,(0.0,0.0,0.0)).
rgbcolor(magenta,(1.0,0.0,1.0)).
rgbcolor(red,(1.0,0.0,0.0)).
rgbcolor(brown,(0.55,0.27,0.7)).
rgbcolor(orange,(1.0,0.5,0.0)).
rgbcolor(yellow,(1.0,1.0,0.0)).
rgbcolor(green,(0.0,1.0,0.0)).
rgbcolor(cyan,(0.0,1.0,1.0)).
rgbcolor(blue,(0.0,0.0,1.0)).
rgbcolor(purple,(0.5,0.0,0.5)).
rgbcolor(pink,(1.0,0.0,0.8)).



visionfield(x,(0.2,0.8)).
visionfield(y,(0.0,0.63)).
visionfield(z,(0.0,0.5)).



% asso(A_ID):t+1 ~ val(A_ID) <- %first time step
% 	\+asso(_):t~=_,
% 	observation(anchor_r(A_ID))~=_.
% asso(A_ID):t+1 ~ val(T_ID) <- %associate anchor with one from previous timestep (often itself) or noise
% 	asso(_):t~=_,
% 	observation(anchor_r(A_ID))~=_,
% 	asso(A_ID):t ~=T_ID.
% asso(A_ID):t+1 ~ val(T_ID) <-
% 	asso(_):t~=_,
% 	observation(anchor_r(A_ID))~=_,
% 	\+asso(A_ID):t ~=_,
% 	T_ID = A_ID.
% asso(A_ID):t+1 ~ val(T_ID) <-
% 	asso(_):t~=_,
% 	\+hidden(A_ID,_):t,
% 	asso(A_ID):t ~= T_ID,
% 	\+observation(anchor_r(A_ID)) ~=_,
% 	pick_occluder(A_ID):t ~=_. %this checks whether there is an occluder present
% asso(A_ID):t+1 ~ val(T_ID) <-
% 	asso(_):t ~=_,
% 	hidden(A_ID,_):t,
% 	asso(A_ID):t~=T_ID.
% asso(A_ID):t+1 ~ val(T_ID) <-
% 	asso(_):t ~=_,
% 	asso(A_ID):t ~= T_ID,
% 	hidden(A_ID, _):t,
% 	observation(anchor_r(A_ID)) ~=_.

%first time step anchor association
asso(A_ID,A_ID):t+1 <-
	\+asso(_,_):t,
	observation(anchor_r(A_ID))~=_.
	%associate anchor with one from previous timestep (often itself)
asso(A_ID,A_ID):t+1 <-
	asso(_,_):t,
	observation(anchor_r(A_ID))~=_.



observed(A_ID):t+1 <-
	observation(anchor_r(A_ID))~=_.

%hidden
% hidden(A_ID,_):t <-
% 	asso(A_ID,_):t,
% 	true,
% 	writeln(true).

% hidden(A_ID):t <-
% 	asso(A_ID,_):t,
% 	true.


%observation position
observation(anchor_r(A_ID)):t+1 ~ val(_) <- %first time step
	\+asso(_,_):t.
%associate anchor with itself or noise (no previous target for this anchor)
observation(anchor_r(A_ID)):t+1 ~ val(_) <-
	asso(_,_):t,
	asso(A_ID,A_ID):t+1.


% observation(anchor_r(A_ID)):t+1 ~ logfinite([W:_]) <- %associate anchor with one from previous timestep (is itself, no data asso done) (previous target for this anchor)
% 	asso(_):t ~=_,
% 	asso(A_ID):t+1 ~= T_ID,
% 	asso(_):t ~= T_ID,
% 	\+hidden(A_ID):t,
% 	rvProposal(A_ID):t+1 ~= [_|W].
/*observation(anchor_r(A_ID)):t+1 ~ logfinite([W:_]) <- %associate anchor with noise (previous target for this anchor) not sure about this rule// think harder
	asso(_):t ~=_,
	asso(A_ID):t ~= T_ID,
	T_ID \= noise,
	asso(A_ID):t+1 ~= noise,
	rvProposal(A_ID):t+1 ~= [_|W].*/
% observation(anchor_r(A_ID)):t+1 ~ logfinite([W:_]) <-
% 	asso(_):t ~=_,
% 	asso(A_ID):t ~= T_ID,
% 	hidden(A_ID, _):t,
% 	asso(A_ID):t+1 ~= T_ID,
% 	rvProposal(A_ID):t+1 ~= [_|W].

observation(anchor_r(A_ID)):t+1 ~ val(_) <- %not good
	writeln('fuck leak'),
	true.

%observation color
observation(anchor_c(_)):t+1 ~ val(_) <-
	true.
%observation bb
observation(anchor_bb(_)):t+1 ~ val(_) <-
	true.

%observation hand
observation(anchor_hand(_)):t+1 ~ val(_) <-
	true.



%position transition
rv(A_ID):t+1 ~  indepGaussians([ ([X,0],Cov), ([Y,0],Cov), ([Z,0],Cov) ]) <-
	\+asso(A_ID,_):t ,
	asso(A_ID,_):t+1,
	observation(anchor_r(A_ID)) ~=(X,Y,Z),
	varQ(VarQ),
	VarQ1 is VarQ/5,
	cov(1,Cov,VarQ1).

% rv(A_ID):t+1 ~  val(RV) <-
% 	asso(A_ID):t+1 ~= T_ID,
% 	hidden(A_ID,_):t,
% 	T_ID\=noise,
% 	rv(A_ID):t ~= RV.
%
%
rv(A_ID):t+1 ~ val(V) <-
	rvProposal(A_ID):t+1 ~= [V|_].

rvProposal(A_ID):t+1 ~ logIndepOptimalProposals([
			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x]),
			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
	asso(A_ID,_):t,
	asso(A_ID,_):t+1,
	observation(anchor_r(A_ID)) ~=(O_x,O_y,O_z),
	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
	varQ(VarQ),
	cov(2,Cov,VarQ),
	deltaT(DeltaT),
	V_x_new is V_x,
	V_y_new is V_y,
	V_z_new is V_z,
	R_x_new is R_x+DeltaT*V_x,
	R_y_new is R_y+DeltaT*V_y,
	R_z_new is R_z+DeltaT*V_z.
%
%
% %getting behind
% rvProposal(A_ID):t+1 ~ logIndepOptimalProposals([
% 			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x_new]),
% 			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
% 			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
% 	asso(A_ID):t+1~=T_ID,
% 	asso(A_ID):t~=T_ID,
% 	\+hidden(A_ID,_):t,
% 	T_ID\=noise,
% 	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
% 	\+observation(anchor_r(A_ID)) ~=_,
% 	pick_occluder(A_ID):t ~= A_ID_occ,
% 	observation(anchor_r(A_ID_occ)) ~=(O_x,O_y,O_z),
% 	varQ(VarQ),
% 	cov(2,Cov,VarQ),
% 	deltaT(DeltaT),
% 	O_x_new is O_x-0.05,
% 	V_x_new is V_x,
% 	V_y_new is V_y,
% 	V_z_new is V_z,
% 	R_x_new is R_x+DeltaT*V_x,
% 	R_y_new is R_y+DeltaT*V_y,
% 	R_z_new is R_z+DeltaT*V_z.

%staying behind
/*rvProposal(A_ID):t+1 ~ logIndepOptimalProposals([
			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x_new]),
			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
	asso(A_ID):t+1~=T_ID,
	asso(A_ID):t~=T_ID,
	hidden(A_ID,A_ID_occ):t,
	T_ID\=noise,
	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
	\+observation(anchor_r(A_ID)) ~=_,
	observation(anchor_r(A_ID_occ)) ~=(O_x,O_y,O_z),
	varQ(VarQ),
	cov(2,Cov,VarQ),
	deltaT(DeltaT),
	/*O_x_new is O_x-0.05,
	V_x_new is V_x,
	V_y_new is V_y,
	V_z_new is V_z,
	R_x_new is R_x+DeltaT*V_x,
	R_y_new is R_y+DeltaT*V_y,
	R_z_new is R_z+DeltaT*V_z.*/



%reappearing from behind
/*rvProposal(A_ID):t+1 ~ logIndepOptimalProposals([
			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x_new]),
			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
	asso(A_ID):t+1~=T_ID,
	asso(A_ID):t~=T_ID,
	hidden(A_ID,A_ID_occ):t,
	T_ID\=noise,
	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
	\+observation(anchor_r(A_ID)) ~=_,
	observation(anchor_r(A_ID_occ)) ~=(O_x,O_y,O_z),
	varQ(VarQ),
	cov(2,Cov,VarQ),
	deltaT(DeltaT),
	O_x_new =O_x-0.05,
	V_x_new is V_x,
	V_y_new is V_y,
	V_z_new is V_z,
	R_x_new is R_x+DeltaT*V_x,
	R_y_new is R_y+DeltaT*V_y,
	R_z_new is R_z+DeltaT*V_z.*/

% rvProposal(A_ID):t+1 ~ logIndepOptimalProposals([
% 			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x]),
% 			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
% 			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
% 	asso(A_ID):t+1~=T_ID,
% 	asso(A_ID):t~=T_ID,
% 	hidden(A_ID,_):t,
% 	T_ID\=noise,
% 	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
% 	observation(anchor_r(A_ID)) ~=(O_x,O_y,O_z),
% 	varQ(VarQ),
% 	cov(2,Cov,VarQ),
% 	deltaT(DeltaT),
% 	V_x_new is V_x,
% 	V_y_new is V_y,
% 	V_z_new is V_z,
% 	R_x_new is R_x+DeltaT*V_x,
% 	R_y_new is R_y+DeltaT*V_y,
% 	R_z_new is R_z+DeltaT*V_z.
%
%
%
%




%behind
/*rv(A_ID):t+1 ~ contUniform([(X3,X4),(XV2,XV2),(Y3,Y4),(YV2,YV2),(Z3,Z4),(ZV2,ZV2)])<-
	asso(_):t~=_,
	\+hidden(A_ID,_):t,
	asso(A_ID):t ~= T_ID,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	\+observation(anchor_r(A_ID))~=_,
	pick_occluder(A_ID):t ~= A_ID_occ,
	rv(A_ID_occ):t ~= (X2,XV2,Y2,YV2,Z2,ZV2),
	observation(anchor_bb(A_ID_occ)) ~= (XBB,YBB,ZBB),
	X3 is X2-0.06,
	X4 is X2-0.03,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB/4,
	Z4 is Z2+ZBB/4.*/







/*
%behind
rv(A_ID):t+1 ~ contUniform([(X3,X4),(XV2,XV2),(Y3,Y4),(YV2,YV2),(Z3,Z4),(ZV2,ZV2)])<-
	asso(_):t~=_,
	\+hidden(A_ID,_):t,
	asso(A_ID):t ~= T_ID,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	\+observation(anchor_r(A_ID))~=_,
	pick_occluder(A_ID):t ~= A_ID_occ,
	observation(anchor_r(A_ID_occ)) ~= (X2,Y2,Z2),
	observation(anchor_bb(A_ID_occ)) ~= (XBB,YBB,ZBB),
	X3 is X2-0.06,
	X4 is X2-0.03,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB/4,
	Z4 is Z2+ZBB/4.*/


%behind
/*
rv(A_ID):t+1 ~ contUniform([(X3,X4),(0,0),(Y3,Y4),(0,0),(Z3,Z4),(0,0)])<-
	hidden(A_ID,A_ID_occ):t,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	rv(A_ID_occ):t~=(X2,XV2,Y2,YV2,Z2,ZV2),
	observation(anchor_bb(A_ID_occ))~=(XBB,YBB,ZBB),
	X3 is X2-0.06,
	X4 is X2-0.03,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB/4,
	Z4 is Z2+ZBB/4.*/

/*
%under
rv(A_ID):t+1 ~ contUniform([(X3,X4),(XV2,XV2),(Y3,Y4),(YV2,YV2),(Z3,Z4),(ZV2,ZV2)])<-
	asso(_):t~=_,
	\+hidden(A_ID,_):t,
	asso(A_ID):t ~= T_ID,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	\+observation(anchor_r(A_ID))~=_,
	pick_occluder(A_ID):t ~= A_ID_occ,
	rv(A_ID_occ):t ~= (X2,XV2,Y2,YV2,Z2,ZV2),
	observation(anchor_bb(A_ID_occ)) ~= (XBB,YBB,ZBB),
	X3 is X2-XBB/2,
	X4 is X2-XBB/2,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB,
	Z4 is Z2-ZBB/2.

%under
rv(A_ID):t+1 ~ contUniform([(X3,X4),(0,0),(Y3,Y4),(0,0),(Z3,Z4),(0,0)])<-
	asso(_):t~=_,
	\+hidden(A_ID,_):t,
	asso(A_ID):t ~= T_ID,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	\+observation(anchor_r(A_ID))~=_,
	pick_occluder(A_ID):t ~= A_ID_occ,
	observation(anchor_r(A_ID_occ)) ~= (X2,Y2,Z2),
	observation(anchor_bb(A_ID_occ)) ~= (XBB,YBB,ZBB),
	X3 is X2-XBB/2,
	X4 is X2-XBB/2,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB,
	Z4 is Z2-ZBB/2.


%under
rv(A_ID):t+1 ~ contUniform([(X3,X4),(XV2,XV2),(Y3,Y4),(YV2,YV2),(Z3,Z4),(ZV2,ZV2)])<-
	hidden(A_ID,A_ID_occ):t,
	rv(A_ID):t~=(X1,_,Y1,_,Z1,_),
	rv(A_ID_occ):t~=(X2,XV2,Y2,YV2,Z2,ZV2),
	observation(anchor_bb(A_ID_occ))~=(XBB,YBB,ZBB),
	X3 is X2-XBB/2,
	X4 is X2-XBB/2,
	Y3 is Y2-YBB/4,
	Y4 is Y2+YBB/4,
	Z3 is Z2-ZBB,
	Z4 is Z2-ZBB/2.

*/

%if observed
color(A_ID,C):t+1 <-
	asso(A_ID,_):t+1,
	observation(anchor_c(A_ID)) ~= C.
%if not observed
color(A_ID,C):t+1 <-
	asso(A_ID,_):t+1,
	\+observation(anchor_c(A_ID))~= _,
	color(A_ID,C):t.


%if observed
bb(A_ID,BB):t+1 ~ val(BB) <-
	asso(A_ID,_):t+1,
	observation(anchor_bb(A_ID)) ~= BB.
%if not observed
bb(A_ID,BB):t+1 <-
	asso(A_ID,_):t+1,
	\+observation(anchor_bb(A_ID))~= _,
	bb(A_ID,BB):t.

is_hand(A_ID):t+1 <-
	observation(anchor_hand(A_ID)) ~= true.





o:-
	N=5,
	init_particle(N),
	step_particle([],[], N, 1.0),
	eval_query_particle((current(color(1,yellow))),N,P0),
	writeln(P0),
	step_particle([],[observation(anchor_c(1))~=yellow,observation(anchor_c(2))~=red, observation(anchor_r(1))~=(0.0,-0.1,0.0), observation(anchor_r(2))~=(0.0,-0.1,0.0)], N, 1.0),
	eval_query_particle((current(color(1,yellow))),N,P1),
	writeln(P1),
	writeln(done).
