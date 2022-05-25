rng('default')  % For reproducibility
failuretime = random('wbl',3,1,15,1);
[f,x] = ecdf(failuretime);
ecdf(failuretime)
[f,x]