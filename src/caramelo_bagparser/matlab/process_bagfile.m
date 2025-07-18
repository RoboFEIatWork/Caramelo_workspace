function process_bagfile(bagPath, outputDir)
    clc;

    if nargin < 2
        error("Uso: process_bagfile(bagPath, outputDir)");
    end

    bagreader = rosbagreader(bagPath);
    bagselect = select(bagreader, 'Topic', '/atwork_commander/task');
    msgs = readMessages(bagselect, 'DataFormat', 'struct');
    task = msgs{1};

    start_data = agrupar_por_ws(task.ArenaStartState);
    target_data = agrupar_por_ws(task.ArenaTargetState);

    % Criar nome do arquivo YAML com base no nome da bag
    outputFile = fullfile(outputDir, "task.yaml");

    % Monta estrutura para exportar
    transitions = struct("start", start_data, "target", target_data);

    % Exportar como YAML usando JSON temporário
    jsonTemp = tempname + ".json";
    fid = fopen(jsonTemp, 'w');
    fwrite(fid, jsonencode(transitions, 'PrettyPrint', true));
    fclose(fid);

    system(sprintf("python3 -c ""import sys, yaml, json; yaml.safe_dump(json.load(open('%s')), open('%s','w'), sort_keys=False)""", jsonTemp, outputFile));
    delete(jsonTemp);

    fprintf("✅ YAML gerado com sucesso: %s\n", outputFile);
end

function lista_ws = agrupar_por_ws(arenaState)
    lista_ws = [];

    for i = 1:length(arenaState)
        ws = arenaState(i);

        if isempty(ws.Objects)
            continue; % pula workstations vazias
        end

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
