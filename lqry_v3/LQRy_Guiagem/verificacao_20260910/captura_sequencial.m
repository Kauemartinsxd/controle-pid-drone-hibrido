function [n_seq, t_hit, d_seq] = captura_sequencial(t, Y, WPs, R_accept)
% CAPTURA_SEQUENCIAL  Capturas na ORDEM da missao: o WP k so' conta se o aviao entrou no circulo
% de R_accept do WP k DEPOIS de capturar o WP k-1. Evita o artefato da metrica "dist min" do
% guiagem_NL.m, que conta o ultimo WP (na origem) em t = 0, antes de o aviao sair do lugar.
N = size(WPs, 1); t_hit = nan(1, N); d_seq = nan(1, N); k0 = 1; n_seq = 0;
for k = 1:N
    d = sqrt((Y(11, k0:end) - WPs(k, 1)).^2 + (Y(12, k0:end) - WPs(k, 2)).^2);
    d_seq(k) = min(d);
    j = find(d <= R_accept, 1);
    if isempty(j), break; end
    n_seq = n_seq + 1; t_hit(k) = t(k0 + j - 1); k0 = k0 + j - 1;
end
end
