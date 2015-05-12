function D = acos_table(x)
    persistent tbl;
    persistent ind;

    if isempty(tbl)
        ind = -1+2/256:2/256:1;
        tbl = acos(ind);
    end
    arr=ones(length(x),1);
    ind_arr=arr*ind;
    
    arr=ones(1,length(ind));
    x_arr=x*arr;
    
    [c index] = min(abs(x_arr - ind_arr),[],2);
    
    D = tbl(index)';