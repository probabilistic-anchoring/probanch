%%% -*- Mode: Prolog; -*-
:- use_module(library(distributionalclause)).
:- use_module(library(dcpf)).


:- use_module(library(lists)).
:- set_options(default).
:- set_inference(backward(lazy)).




builtin(rgbcolor(_,_)).
builtin(deltaT(_)).
builtin(varQ(_)).
builtin(cov(_,_,_)).



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



observed_list(A_List):t+1 <-
   findall_forward(A_ID, (observation(anchor_r(A_ID))~=_), A_List).

observed(A_ID):t+1 <-
   observed_list(A_List):t+1,
   member(A_ID,A_List).


anchor(A_ID):t+1 <-
	\+anchor(_):t,
	observation(anchor_r(A_ID))~=_.
anchor(A_ID):t+1 <-
	anchor(_):t,
	observation(anchor_r(A_ID))~=_.
anchor(A_ID):t+1 <-
	anchor(_):t,
	anchor(A_ID):t,
   hidden(A_ID,_):t+1.

hidden(A_ID,A_ID_occ):t+1 <-
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   occluded_by(A_ID,A_ID_occ):t+1.
hidden(A_ID,A_ID_occ):t+1 <-
   \+observed(A_ID):t+1,
   hidden(A_ID,A_ID_occ):t,
   anchor(A_ID_occ):t+1.


occluded_by(A_ID,A_ID_hand):t+1 <-
   in_hand(A_ID,A_ID_hand):t+1.


in_hand(A_ID,A_ID_hand):t+1 <-
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   pick_hand(A_ID):t+1 ~= A_ID_hand.
in_hand(A_ID,A_ID_hand):t+1 <-
   in_hand(A_ID,A_ID_hand):t,
   \+observed(A_ID):t,
   \+observed(A_ID):t+1,
   anchor(A_ID):t+1,
   anchor(A_ID_hand):t+1.



pick_hand(A_ID):t+1 ~ uniform(Hands) <-
   anchor(A_ID):t,
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   \+is_hand(A_ID):t,
   \+hidden(A_ID,_):t,
   rv(A_ID):t ~= (X1,_,Y1,_,Z1,_),
   findall_forward(H, (is_hand(H):t+1, rv(H):t+1~=(XH,_,YH,_,ZH,_), D is sqrt((X1-XH)^2+(Y1-YH)^2), D<0.3, Z1<ZH), Hands),
   caffe(A_ID):t ~= Caffe,
   \+Hands=[].




% observation position
observation(anchor_r(A_ID)):t+1 ~ val(_) <- %first time step
	\+anchor(_):t.
observation(anchor_r(A_ID)):t+1 ~ val(_) <-
	anchor(_):t,
   \+anchor(A_ID):t,
	anchor(A_ID):t+1.
% observation(anchor_r(A_ID)):t+1 ~ val(_) <-
% 	anchor(_):t,
%    hidden(A_ID,_):t+1,
% 	anchor(A_ID):t+1.

observation(anchor_r(A_ID)):t+1 ~ logfinite([W:_]) <-
	anchor(_):t,
   anchor(A_ID):t,
	anchor(A_ID):t+1,
	rvProposal(_,A_ID):t+1 ~= [_|W].
observation(anchor_r(A_ID)):t+1 ~ val(_) <- %not good
   anchor(_):t,
   writeln(A_ID),
   observation(anchor_caffe(A_ID))~=Caffe,
   writeln(Caffe),
	writeln('fuck leak'),
	true.

%observation color
observation(anchor_c(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation caffe
observation(anchor_caffe(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation bb
observation(anchor_bb(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.
%observation hand
observation(anchor_hand(A_ID)):t+1 ~ val(_) <-
	anchor(A_ID):t+1.



%position transition
rv(A_ID):t+1 ~  indepGaussians([ ([X,0],Cov), ([Y,0],Cov), ([Z,0],Cov) ]) <-
	\+rv(A_ID,_):t,
	anchor(A_ID):t+1,
	observation(anchor_r(A_ID)) ~=(X,Y,Z),
	varQ(VarQ),
	VarQ1 is VarQ/5,
	cov(1,Cov,VarQ1).

rv(A_ID):t+1 ~ val(V) <-
	anchor(A_ID):t,
   observation(anchor_r(A_ID))~=_,
	anchor(A_ID):t+1,
	rvProposal('observed',A_ID):t+1 ~= [V|_].

rv(A_ID):t+1 ~ val(V) <-
	observed(A_ID):t,
   anchor(A_ID):t,
   \+observed(A_ID):t+1,
   anchor(A_ID):t+1,
   \+is_hand(A_ID):t+1,
   in_hand(A_ID,_):t+1,
	rvProposal('to_hand',A_ID):t+1 ~= [V|_].

rv(A_ID):t+1 ~ val(V) <-
	\+observed(A_ID):t,
   anchor(A_ID):t,
   \+observed(A_ID):t+1,
   anchor(A_ID):t+1,
   in_hand(A_ID,_):t+1,
	rvProposal('in_hand',A_ID):t+1 ~= [V|_].



% directly observed
rvProposal('observed',A_ID):t+1 ~ logIndepOptimalProposals([
			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x]),
			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
	observation(anchor_r(A_ID)) ~= (O_x,O_y,O_z),
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

% getting in hand
rvProposal('to_hand',A_ID):t+1 ~ val(RVP) <-
   rvProposal('in_hand',A_ID):t+1 ~= RVP.
% staying in hand
rvProposal('in_hand',A_ID):t+1 ~ logIndepOptimalProposals([
			([R_x_new,V_x_new],Cov, [1,0],[0.0001],[O_x]),
			([R_y_new,V_y_new],Cov, [1,0],[0.0001],[O_y]),
			([R_z_new,V_z_new],Cov, [1,0],[0.0001],[O_z])]) <-
	rv(A_ID):t ~= (R_x,V_x,R_y,V_y,R_z,V_z),
	in_hand(A_ID,A_ID_hand):t+1,
	observation(anchor_r(A_ID_hand)) ~=(O_x,O_y,O_z),
	varQ(VarQ),
	cov(2,Cov,VarQ),
	deltaT(DeltaT),
	V_x_new is V_x,
	V_y_new is V_y,
	V_z_new is V_z,
	R_x_new is R_x+DeltaT*V_x,
	R_y_new is R_y+DeltaT*V_y,
	R_z_new is R_z+DeltaT*V_z.


%if observed
color(A_ID):t+1 ~ val(C) <-
	anchor(A_ID):t+1,
	observation(anchor_c(A_ID)) ~= C.
%if not observed
color(A_ID):t+1 ~ val(C) <-
	anchor(A_ID):t+1,
	color(A_ID):t ~= C.

caffe(A_ID):t+1 ~ val(Caffe) <-
	anchor(A_ID):t+1,
	observation(anchor_caffe(A_ID)) ~= Caffe.
%if not observed
caffe(A_ID):t+1 ~ val(Caffe) <-
	anchor(A_ID):t+1,
	caffe(A_ID):t ~= Caffe.

%if observed
bb(A_ID):t+1 ~ val(BB) <-
	anchor(A_ID):t+1,
	observation(anchor_bb(A_ID)) ~= BB.
%if not observed
bb(A_ID):t+1 ~ val(BB) <-
	anchor(A_ID):t+1,
	bb(A_ID):t ~= BB.

is_hand(A_ID):t+1 <-
	observation(anchor_hand(A_ID)) ~= true.
