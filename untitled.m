clc;
clear;

% === Caminhos ===
bagPath = fullfile(getenv('HOME'), 'BTT2_25.bag');  % ajuste o nome se necessário
outputDir = fullfile(getenv('HOME'), 'Caramelo_workspace', 'src', 'caramelo_tasks');

if ~isfile(bagPath)
    error("❌ Arquivo .bag não encontrado: %s", bagPath);
end

fprintf("📦 Usando arquivo .bag: %s\n", bagPath);
fprintf("📁 Salvando YAML em: %s\n", outputDir);

% === Leitura da bag e extração dos dados ===
bagreader = rosbagreader(bagPath);
bagselect = select(bagreader, 'Topic', '/atwork_commander/task');
msgs = readMessages(bagselect, 'DataFormat', 'struct');

if isempty(msgs)
    error("❌ Nenhuma mensagem encontrada no tópico /atwork_commander/task");
end

task = msgs{1};

% === Agrupamento por Workstation ===
lista_ws_start = agrupar_por_ws(task.ArenaStartState);
lista_ws_target = agrupar_por_ws(task.ArenaTargetState);

% === Estrutura final para exportação ===
transitions = struct("start", lista_ws_start, "target", lista_ws_target);

% === Exportar como YAML via conversão temporária JSON → YAML ===
jsonTemp = tempname + ".json";
fid = fopen(jsonTemp, 'w');
fwrite(fid, jsonencode(transitions, 'PrettyPrint', true));
fclose(fid);

outputFile = fullfile(outputDir, "task.yaml");
cmd = sprintf("python3 -c ""import sys, yaml, json; yaml.safe_dump(json.load(open('%s')), open('%s','w'), sort_keys=False)""", jsonTemp, outputFile);
system(cmd);
delete(jsonTemp);

fprintf("✅ YAML gerado com sucesso: %s\n", outputFile);

% === Função interna ===
function lista_ws = agrupar_por_ws(arenaState)
    lista_ws = [];

    for i = 1:length(arenaState)
        ws = arenaState(i);
        ws_entry = struct();
        ws_entry.ws = string(ws.Name);
        ws_entry.type = string(ws.Type);

        obj_list = [];
        for j = 1:length(ws.Objects)
            obj = ws.Objects(j);
            obj_list = [obj_list, struct( ...
                'id', double(obj.Object_ - 10), ...
                'decoy', logical(obj.Decoy) ...
            )];
        end

        ws_entry.objects = obj_list;
        lista_ws = [lista_ws, ws_entry];
    end
end