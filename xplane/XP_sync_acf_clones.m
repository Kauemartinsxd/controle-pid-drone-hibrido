% XP_sync_acf_clones.m — copia o DH-Lon-REV-03.acf ATIVO para todos os
% DH-Lon-REV-03_zz_clone*.acf da pasta do X-Plane (e confere md5).
% Motivo (2026-09-02): o reload automatico clica numa LINHA FIXA do dialogo
% Open Aircraft; com varios DH-Lon-REV-03*.acf, a linha pode ser um backup.
% Com clones identicos, qualquer linha carrega o ativo. Backups verdadeiros:
% Dissertacao_Mestrado\acf_backups_<data>\. Rode apos editar no Plane Maker.
acfDir = 'C:\Users\kaue\Documents\Dissertacao_Mestrado\X-Plane 9\X-Plane 9\Aircraft\Radio Control';
ativo  = fullfile(acfDir, 'DH-Lon-REV-03.acf');
h = @(f) lower(char(java.lang.String(sprintf('%02x', typecast(org.apache.commons.codec.digest.DigestUtils.md5(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(f))), 'uint8')))));
try, h0 = h(ativo); catch, h0 = ''; end
L = dir(fullfile(acfDir, 'DH-Lon-REV-03_zz_clone*.acf'));
for k = 1:numel(L)
    copyfile(ativo, fullfile(L(k).folder, L(k).name));
end
fprintf('XP_sync_acf_clones: %d clones sincronizados com o ativo', numel(L));
if ~isempty(h0), fprintf(' (md5 %s)', h0(1:8)); end
fprintf('.\n');
