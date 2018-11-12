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
varQ(0.001).

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

observed_list:t+1 ~ val(A_List) <-
   findall_forward(A_ID, (observation(anchor_r(A_ID))~=_), A_List).

observed(A_ID):t+1 <-
   observed_list:t+1 ~= A_List,
   member(A_ID,A_List).


box(A_ID):t+1 <-
   observation(anchor_r(A_ID))~=_,
   \+box(A_ID):t.
box(A_ID):t+1 <-
   box(A_ID):t.

small_box(A_ID):t+1 <-
   small_box(A_ID):t.
small_box(A_ID):t+1 <-
   box(A_ID):t+1,
   observation(anchor_c(A_ID))~=C,
   C==green.

big_box(A_ID):t+1 <-
   big_box(A_ID):t.
big_box(A_ID):t+1 <-
   box(A_ID):t+1,
   observation(anchor_c(A_ID))~=C,
   C==pink.




hidden(A_ID,A_ID_BigBox):t+1 <-
   under_of(A_ID,A_ID_BigBox):t+1.


under_of(A_ID,A_ID_BigBox):t+1 <-
   small_box(A_ID):t+1,
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   pick_BigBox(A_ID):t+1 ~= A_ID_BigBox.
under_of(A_ID,A_ID_BigBox):t+1 <-
   small_box(A_ID):t+1,
   under_of(A_ID,A_ID_BigBox):t,
   \+observed(A_ID):t+1.

pick_BigBox(A_ID):t+1 ~ uniform(BigBoxes) <-
   small_box(A_ID):t+1,
   observed(A_ID):t,
   \+observed(A_ID):t+1,
   \+hidden(A_ID,_):t,
   rv(A_ID):t ~= (X1,_,Y1,_,Z1,_),
   findall_forward(BB, (big_box(BB):t+1, rv(BB):t+1~=(XBB,_,YBB,_,ZBB,_),  D is sqrt((X1-XBB)^2+(Y1-YBB)^2), D<0.2), BigBoxes),
   \+BigBoxes=[].





% observation position
observation(anchor_r(A_ID)):t+1 ~ val(_) <-
	box(A_ID):t+1.
observation(anchor_r(A_ID)):t+1 ~ val(_) <- %not good
   anchor(_):t,
	writeln('fuck leak'),
	true.




%observation color
observation(anchor_c(A_ID)):t+1 ~ val(_) <-
   true.
%observation caffe
observation(anchor_caffe(A_ID)):t+1 ~ val(_) <-
   true.
%observation bb
observation(anchor_bb(A_ID)):t+1 ~ val(_) <-
   true.




%position transition
rv(A_ID):t+1 ~  indepGaussians([ ([X,0],Cov), ([Y,0],Cov), ([Z,0],Cov) ]) <-
	observation(anchor_r(A_ID)) ~= (X,Y,Z),
	\+box(A_ID):t,
	box(A_ID):t+1,
	varQ(VarQ),
	VarQ1 is VarQ/5,
	cov(1,Cov,VarQ1).

%replace this one with optimal proposal?
rv(A_ID):t+1 ~  indepGaussians([ ([X,0],Cov), ([Y,0],Cov), ([Z,0],Cov) ]) <-
	observation(anchor_r(A_ID)) ~= (X,Y,Z),
   box(A_ID):t,
	box(A_ID):t+1,
	varQ(VarQ),
	VarQ1 is VarQ/5,
	cov(1,Cov,VarQ1).


rv(A_ID):t+1 ~ val(RV) <-
   big_box(A_ID):t+1,
	\+observed(A_ID):t+1,
   rv(A_ID):t ~= RV.


rv(A_ID):t+1 ~ val(RV) <-
   small_box(A_ID):t+1,
   \+observation(anchor_r(A_ID)) ~= _,
   under_of(A_ID,A_ID_BigBox):t+1,
   rv(A_ID_BigBox):t+1 ~= RV.






color(A_ID):t+1 ~ val(C) <-
	observation(anchor_c(A_ID)) ~= C.
color(A_ID):t+1 ~ val(C) <-
	box(A_ID):t+1,
	color(A_ID):t ~= C.

caffe(A_ID):t+1 ~ val(Caffe) <-
	observation(anchor_caffe(A_ID)) ~= Caffe.
caffe(A_ID):t+1 ~ val(Caffe) <-
	box(A_ID):t+1,
	caffe(A_ID):t ~= Caffe.
