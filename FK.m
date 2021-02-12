function T=FK(q,L)
T=(Tz(L(1))*Rz(q(1))*Tx(L(2))*Rz(q(2))*Tx(L(3))*Tz(q(3))*Tz(-L(4))*Rz(q(4)));
end