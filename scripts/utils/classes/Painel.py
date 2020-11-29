import time


class Painel:
    def __init__(self, percent_gas, ajuste_zero):
        self._gas = percent_gas
        self._ajuste_zero = ajuste_zero

    @property
    def ajuste_zero(self):
        return self._ajuste_zero

    @property
    def gas(self):
        return self._gas

    def __str__(self):
        print(18 * "*")
        if self.gas <= 55 and self.gas >= 45:
            print("-> Percentual do gas: {}   |||   EM CONFORMIDADE".format(self.gas))
        else:
            print(
                "-> Percentual do gas: {}   |||   FORA DE CONFORMIDADE".format(self.gas)
            )

        time.sleep(10)
        print("Aguardando sinalizacao do ajuste de zero\n" + 12 * "-")
        time.sleep(33)

        if self.ajuste_zero <= 5 and self.ajuste_zero >= -5:
            print(
                "-> Ajuste de zero: {}   |||   EM CONFORMIDADE".format(self.ajuste_zero)
            )
        else:
            print(
                "-> Ajuste de zero: {}   |||   FORA DE CONFORMIDADE".format(
                    self.ajuste_zero
                )
            )

        time.sleep(10)
        print(18 * "*")
        return "\n"
