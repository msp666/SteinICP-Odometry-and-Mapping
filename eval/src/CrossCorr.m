function xCorr = CrossCorr(OdomError, Cov)
    OdomError_unbiased = OdomError - mean(OdomError, 1);
    Cov_unbiased = Cov - mean(Cov, 1);

    % if length(OdomError_unbiased) ~= length(Cov_unbiased)
    %     error("length of Error and Covariance is not consistent");
    % end
    max_lag = int64(length(OdomError_unbiased) / 2);

    for i = 1:6 
        [xCorr(:,i+1), lags] = xcorr(OdomError_unbiased(:,i), Cov_unbiased(:,i), max_lag);
    end
    xCorr(:,1) = lags;
end