/**
 * Name: PelotasEnchente
 * Author: Ulisses, Leticia e Agatha
 * Versão: 1.0.0 (Falta ainda retirar algumas variaveis desnecessarias, existe espaço para melhorar a performance tbm)
 * Descrição: Simulação de inundação na cidade de Pelotas
 */

model PelotasEnchente

global {
   // Shapefile for the river
   file river_shapefile <- file("../includes/PelotasAguasRecorte.shp");   
   file limite <- file("../includes/PelotasTOTAL.shp");
   // Shapefile for the buildings
   file roads_shapefile <- file("../includes/RuasPelotas.shp");
   
   // Data elevation file
   file dem_file <- file("../includes/PelotasRelevo3.tif");
   // Diffusion rate
   float diffusion_rate <- 0.6;
    
   // Shape of the environment using the dem file
   geometry shape <- envelope(limite);
   
   // List of the drain and river cells
   list<cell> drain_cells;
   list<cell> river_cells;
   list<cell> celulasSurgeAgua;
   list<cell> celulas_manuais;
   bool usar_celulas_manuais <- true;
   bool simular_chuva <- false;
   float limiar_inundacao <- 0.1;
   
   // Novas variáveis para otimização
   list<cell> active_cells;    // Células que precisam ser processadas
   list<cell> stable_cells;    // Células que não precisam mais ser verificadas
   bool use_optimized_flow <- true; // Parâmetro para ativar/desativar otimização
   
   float step <- 1#h;
   
init {
   // Initialization of the cells
   do init_cells;
   
   // Inicializar listas de otimização
   active_cells <- [];
   stable_cells <- [];
   
   // Defina as células onde a água surge primeiro
   do celulas_surge_agua;
   ask celulasSurgeAgua {
      conectada <- true;
      active_cells <- active_cells + [self]; // Adicionar às células ativas
   }
   
   // Initialization of the water cells apenas na área de origem
   do init_water;
   
   // Initialization of the river cells
   river_cells <- cell where (each.is_river);
   ask river_cells {
      active_cells <- active_cells + [self]; // Adicionar rios às células ativas
   }
   
   // Initialization of the obstacles
   create road from: roads_shapefile;
   
   // Set the color of each cell
   ask cell {
      do update_color;
   }
}

// Action to initialize the altitude value of the cell according to the dem file
action init_cells {
   ask cell {
      altitude <- grid_value;
      neighbour_cells <- (self neighbors_at 1);
      last_water_height <- water_height;
   }
}

// Action to initialize the water cells according to the river shape file and the drain
action init_water {
   geometry river <- geometry(river_shapefile);
   ask cell overlapping river {
      water_height <- 10.0;
      is_river <- true;
      inundada <- true;
   }
}
   
action celulas_surge_agua {
   float x_minimo <- 350.0;
   float y_minimo <- 130.0;
   celulasSurgeAgua <- cell where (each.grid_x >= x_minimo and each.grid_y >= y_minimo);
}

// Reflex to add water among the water cells
reflex adding_input_water {
   if (usar_celulas_manuais) {
      ask celulasSurgeAgua where (each.conectada) {
         water_height <- water_height + (rnd(100)/100);
         // Se recebeu água, tornar-se ativa
         if (not (self in active_cells)) {
            active_cells <- active_cells + [self];
            // Remover de stable_cells se estiver lá
            if (self in stable_cells) {
               stable_cells <- list<cell>(stable_cells) - [self];
            }
         }
      }
   } else if (simular_chuva) {
      ask river_cells where (each.conectada) {
         water_height <- water_height + (rnd(100)/100);
         // Se recebeu água, tornar-se ativa
         if (not (self in active_cells)) {
            active_cells <- active_cells + [self];
            // Remover de stable_cells se estiver lá
            if (self in stable_cells) {
               stable_cells <- list<cell>(stable_cells) - [self];
            }
         }
      }
   }
}

// Reflex to flow the water according to the altitude and the obstacle
reflex flowing {
   if (use_optimized_flow) {
      do optimized_flow;
   } else {
      do legacy_flow;
   }
}

// Implementação otimizada do fluxo de água
action optimized_flow {
   // Lista temporária para novas células ativas
   list<cell> new_active_cells <- [];
   // Lista de células a remover da lista ativa
   list<cell> to_remove_from_active <- [];
   
   // Processar cada célula ativa
   ask active_cells {
      bool had_activity <- false;
      
      // Verificar se há água para distribuir
      if (water_height > 0 and conectada) {
         // Encontrar vizinhos que podem receber água
         list<cell> floodable_neighbors <- list<cell>(neighbour_cells) where (each.can_be_flooded(self));
         
         if (not empty(floodable_neighbors)) {
            had_activity <- true;
            
            // Ordenar vizinhos por potencial de inundação (mais baixos primeiro)
            floodable_neighbors <- floodable_neighbors sort_by (each.altitude + each.water_height);
            
            // Calcular quantidade total de água para distribuir
            float total_water_to_flow <- water_height * diffusion_rate;
            float water_distributed <- 0.0;
            
            // Distribuir água para os vizinhos
            loop neighbor over: floodable_neighbors {
               if (water_distributed < total_water_to_flow) {
                  float available_capacity <- (altitude + water_height) - (neighbor.altitude + neighbor.water_height);
                  
                  if (available_capacity > 0) {
                     float water_to_flow <- min([available_capacity, total_water_to_flow - water_distributed]);
                     
                     water_height <- water_height - water_to_flow;
                     ask neighbor {
                        water_height <- water_height + water_to_flow;
                        
                        // Marcar como inundada se ultrapassar o limiar
                        if (water_height > limiar_inundacao and not inundada) {
                           inundada <- true;
                        }
                        
                        // Adicionar à lista de novas células ativas
                        if (not (self in new_active_cells) and not (self in active_cells)) {
                           new_active_cells <- new_active_cells + [self];
                        }
                     }
                     
                     water_distributed <- water_distributed + water_to_flow;
                  }
               }
            }
         }
      }
      
      // Atualizar último valor de água conhecido
      last_water_height <- water_height;
      
      // Se não houve atividade, marcar para remoção
      if (not had_activity and water_height <= 0) {
         to_remove_from_active <- to_remove_from_active + [self];
      }
   }
   
   // Atualizar lista de células ativas
   active_cells <- (list<cell>(active_cells) - to_remove_from_active) + new_active_cells;
   
   // Adicionar células removidas à lista de células estáveis
   stable_cells <- stable_cells + to_remove_from_active;
}

// Implementação legada do fluxo de água (mantida para comparação)
action legacy_flow {
   ask (cell where (each.water_height > 0 or each.conectada)) {
      already <- false;
   }
   
   list<cell> cells_to_process <- cell where (each.water_height > 0 or each.conectada);
   ask (cells_to_process sort_by (each.altitude + each.water_height)) {
      do flow;
   }
}

// Reflex to update the color of the cell
reflex update_cell_color {
   ask cell {
      do update_color;
   }
}

// Reflex for the drain cells to drain water
reflex draining {
   ask drain_cells {
      water_height <- 0.0;
   }
}
   
species road {
   rgb color <- #black;
   aspect base {
      draw shape color: color;
   }
}
   
// Grid cell to discretize space, initialized using the dem file
grid cell file: dem_file neighbors: 8 frequency: 0 use_regular_agents: false use_individual_shapes: false use_neighbors_cache: false schedules: [] {
   // Altitude of the cell
   float altitude;
   // Height of the water in the cell
   float water_height <- 0.0 min: 0.0;
   // Height of the cell
   //float height;
   // List of the neighbour cells
   list<cell> neighbour_cells;
   // Boolean to know if it is a drain cell
   bool is_drain <- false;
   // Boolean to know if it is a river cell
   bool is_river <- false;
   // List of all the obstacles overlapping the cell
   bool inundada <- false;
   bool conectada <- true;
   
   // Height of the obstacles
   float obstacle_height <- 0.0;
   bool already <- false;
   
   // Nova propriedade para otimização
   float last_water_height <- 0.0;
   
   // Action to flow the water (método legado)
   action flow {
      if (conectada and water_height > 0 and not already) {
         already <- true;
         
         // Calcular direção preferencial de fluxo
         cell target_cell <- nil;
         float max_difference <- 0;
         
         loop neighbor over: neighbour_cells {
            if (neighbor.conectada) {
               float difference <- (altitude + water_height) - (neighbor.altitude + neighbor.water_height);
               if (difference > max_difference) {
                  max_difference <- difference;
                  target_cell <- neighbor;
               }
            }
         }
         
         // Transferir água se encontrou direção
         if (target_cell != nil and max_difference > 0) {
            float agua_a_fluir <- min([water_height * diffusion_rate, max_difference * 0.5]);
            water_height <- water_height - agua_a_fluir;
            ask target_cell {
               water_height <- water_height + agua_a_fluir;
               if (water_height > limiar_inundacao) {
                  inundada <- true;
               }
            }
         }
      }
   }
   
   // Método para verificar se esta célula pode ser inundada por outra
   bool can_be_flooded(cell other) {
      return (conectada and (altitude + water_height) < (other.altitude + other.water_height));
   }
   
   // Update the color of the cell
   action update_color {
      if (inundada) {
         // Usar gradiente de azul baseado na profundidade
         float profundidade <- water_height / 5.0;
         profundidade <- min(1.0, profundidade);
         color <- rgb(0, 0, 100 + int(155 * profundidade));
      } else if (is_river) {
         color <- rgb(0, 100, 255);
      } else {
         // Gradiente de altitude mais informativo
         int intensidade <- 255 - int((altitude - 0) / (100 - 0) * 100);
         intensidade <- max(150, min(255, intensidade));
         color <- rgb(intensidade, intensidade, intensidade);
      }
   }
}

}

experiment Run type: gui {
   
   //parameter "Taxa de difusão" var: diffusion_rate category: "Simulação" min: 0.1 max: 1.0;
   
   output {
      display map type: 3d {
         grid cell triangulation: true;
         species road refresh: false;
         species road aspect: base;
      }
      monitor "Células inundadas" value: (cell count (each.inundada));
   }
}