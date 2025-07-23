#include <stdio.h>

// --- Constantes da Simulação ---
// O uso de #define permite fácil ajuste dos parâmetros da simulação.
#define NUMERO_TOTAL_DE_VOLTAS 100
#define SUSPENSAO_INICIAL 100.0f
#define COMBUSTIVEL_INICIAL 100.0f
#define TEMPERATURA_INICIAL_MOTOR 80.0f

#define DESGASTE_SUSPENSAO_POR_VOLTA 2.0f
#define CONSUMO_COMBUSTIVEL_POR_VOLTA 1.5f
#define AUMENTO_TEMP_MOTOR_POR_VOLTA 1.0f

#define PENALIDADE_SUSPENSAO 3.0f
#define PENALIDADE_TEMPERATURA_MOTOR 5.0f
#define INTERVALO_PENALIDADE 10

#define LIMITE_ALERTA_SUSPENSAO 20.0f
#define LIMITE_ALERTA_MOTOR 115.0f

#define INTERVALO_RELATORIO 20

// --- Estrutura de Dados do Veículo ---
// Encapsula o estado do veículo em uma única entidade lógica
typedef struct {
    float suspensao;
    float combustivel;
    double motor_temp;
    // Flags para garantir que os alertas sejam exibidos apenas uma vez.
    int alerta_suspensao_emitido;
    int alerta_motor_emitido;
} BajaVehicle;

// --- Protótipos das Funções ---
// Declarações antecipadas das funções para organizar o código
void initializeBaja(BajaVehicle *carro);
void runLap(BajaVehicle *carro);
void applyPeriodicPenalty(BajaVehicle *carro);
void checkSystemAlerts(BajaVehicle *carro);
void printStatusReport(const BajaVehicle *carro, int volta);
void printFinalReport(const BajaVehicle *carro, int volta_final, int corrida_concluida);

// --- Função Principal ---
int main() {
    BajaVehicle carro_imperador;
    int corrida_concluida = 1; // Flag para indicar se a corrida terminou normalmente.

    initializeBaja(&carro_imperador);

    printf("### INÍCIO DA SIMULAÇÃO DE ENDURO - EQUIPE IMPERADOR UTFPR ###\n\n");
    printf("Condições Iniciais:\n");
    printStatusReport(&carro_imperador, 0);

    // O laço 'for' controla o progresso da simulação, volta a volta
    for (int volta = 1; volta <= NUMERO_TOTAL_DE_VOLTAS; ++volta) {
        // Aplica o desgaste e consumo padrão da volta.
        runLap(&carro_imperador);

        // A cada 10 voltas, aplica uma penalidade.
        // O operador de módulo (%) é uma forma elegante de agendar eventos periódicos.
        if (volta % INTERVALO_PENALIDADE == 0) {
            applyPeriodicPenalty(&carro_imperador);
            printf("\n>>> Penalidade aplicada na volta %d! <<<\n", volta);
        }

        // Verifica condições de alerta.
        checkSystemAlerts(&carro_imperador);

        // A cada 20 voltas, exibe um relatório completo.
        if (volta % INTERVALO_RELATORIO == 0) {
            printStatusReport(&carro_imperador, volta);
        }

        // Verifica condições de falha que encerram a corrida.
        if (carro_imperador.suspensao <= 0.0f || carro_imperador.combustivel <= 0.0f) {
            corrida_concluida = 0; // Marca que a corrida terminou por falha.
            printFinalReport(&carro_imperador, volta, corrida_concluida);
            break; // O comando 'break' interrompe o laço imediatamente
        }
    }

    // Se o laço completou todas as iterações, a corrida foi concluída com sucesso.
    if (corrida_concluida) {
        printFinalReport(&carro_imperador, NUMERO_TOTAL_DE_VOLTAS, corrida_concluida);
    }

    return 0;
}

// --- Implementação das Funções ---

// Inicializa o estado do veículo para as condições de início de corrida.
void initializeBaja(BajaVehicle *carro) {
    carro->suspensao = SUSPENSAO_INICIAL;
    carro->combustivel = COMBUSTIVEL_INICIAL;
    carro->motor_temp = TEMPERATURA_INICIAL_MOTOR;
    carro->alerta_suspensao_emitido = 0;
    carro->alerta_motor_emitido = 0;
}

// Simula o desgaste e consumo de uma única volta.
void runLap(BajaVehicle *carro) {
    carro->suspensao -= DESGASTE_SUSPENSAO_POR_VOLTA;
    carro->combustivel -= CONSUMO_COMBUSTIVEL_POR_VOLTA;
    carro->motor_temp += AUMENTO_TEMP_MOTOR_POR_VOLTA;
}

// Aplica as penalidades de desgaste extra na suspensão e aquecimento do motor.
void applyPeriodicPenalty(BajaVehicle *carro) {
    carro->suspensao -= PENALIDADE_SUSPENSAO;
    carro->motor_temp += PENALIDADE_TEMPERATURA_MOTOR;
}

// Verifica e exibe alertas se os limiares críticos forem ultrapassados.
void checkSystemAlerts(BajaVehicle *carro) {
    if (carro->suspensao < LIMITE_ALERTA_SUSPENSAO &&!carro->alerta_suspensao_emitido) {
        printf("\n!!! ALERTA: Nível da suspensão crítico (%.1f%%)!!!\n", carro->suspensao);
        carro->alerta_suspensao_emitido = 1; // Evita alertas repetidos.
    }
    if (carro->motor_temp > LIMITE_ALERTA_MOTOR &&!carro->alerta_motor_emitido) {
        printf("\n!!! ALERTA: Temperatura do motor excessiva (%.1f°C)!!!\n", carro->motor_temp);
        carro->alerta_motor_emitido = 1; // Evita alertas repetidos.
    }
}

// Imprime um relatório de status formatado.
void printStatusReport(const BajaVehicle *carro, int volta) {
    printf("\n--- RELATÓRIO DA VOLTA %d ---\n", volta);
    printf("=================================\n");
    printf("Suspensão....: %.2f%%\n", carro->suspensao > 0? carro->suspensao : 0.0f);
    printf("Combustível..: %.2f%%\n", carro->combustivel > 0? carro->combustivel : 0.0f);
    printf("Motor........: %.2f°C\n", carro->motor_temp);
    printf("=================================\n\n");
}

// Imprime o relatório final, indicando o motivo do término da corrida.
void printFinalReport(const BajaVehicle *carro, int volta_final, int corrida_concluida) {
    printf("\n##################################################\n");
    printf("### FIM DA SIMULAÇÃO ###\n");
    printf("##################################################\n");

    if (corrida_concluida) {
        printf("\nCORRIDA CONCLUÍDA COM SUCESSO!\n");
        printf("O veículo completou as %d voltas.\n", NUMERO_TOTAL_DE_VOLTAS);
    } else {
        printf("\nFALHA CRÍTICA NA VOLTA %d!\n", volta_final);
        if (carro->suspensao <= 0.0f) {
            printf("Motivo: Falha total da suspensão.\n");
        } else if (carro->combustivel <= 0.0f) {
            printf("Motivo: Combustível esgotado.\n");
        }
    }

    printf("\n--- ESTADO FINAL DO VEÍCULO ---\n");
    printStatusReport(carro, volta_final);
}