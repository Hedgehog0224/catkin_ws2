from math import pi, cos, sin
from numpy import zeros, matmul, deg2rad, array, ndarray
from typing import Union, Any, Optional

class data():
    def __init__(self, a1: Optional [int]=10, s1: Optional [int]=36, l1: Optional [int]=178, l2: Optional [int]=159.8, s4: Optional [int]=10, s5: Optional [int]=135, q=array([0,0,0,0,0])) -> None:
        """
        Класс для хранения данных о манипуляторе
        """
        self.slovar = dict()
        self.slovar["a1"] = a1
        self.slovar["s1"] = s1
        self.slovar["l1"] = l1
        self.slovar["l2"] = l2
        self.slovar["s4"] = s4
        self.slovar["s5"] = s5
        self.slovar["q"] = q
        self.slovar["DH"] = self.calculationOfDH(q)

    def calculationOfDH(self, new_q: array):
        """
        Вычисление параметров Денавита-Хартенберга
        """
        self.slovar["q"] = new_q
        DH = [[self.slovar["a1"],  self.slovar["l1"], self.slovar["l2"], 0,                 0],                 # a     |0
             [pi/2,                0,                 0,                 pi/2,              0],                 # alfa  |1
             [self.slovar["s1"],   0,                 0,                 self.slovar["s4"], self.slovar["s5"]], # d     |2
             [new_q[0],            new_q[1],          new_q[2],          new_q[3]+pi/2,     new_q[4]]]          # theta |3
        self.slovar["DH"] = DH
        return DH

    def get_data(self, *names: str) -> Union [float, int, list]:
        """
        Возвращает значение из общего словаря по его имени: a1, s1, l1, l2, s4, s5, q, DH
        """
        res = []
        for i in names:
            res.append(self.slovar[i])
        return res

class baseCalculations(data):
    def __init__(self, a1: Optional [int]=10, s1: Optional [int]=36, l1: Optional [int]=178, l2: Optional [int]=159.8, s4: Optional [int]=10, s5: Optional [int]=135, q=array([0,0,0,0,0])) -> None:
        """
        Класс для базовых вычислений для манипулятора
        """
        super().__init__(a1, s1, l1, l2, s4, s5, q)
        self.T = zeros((5,4,4))
        for i in range(5):
            a = 0
            alfa = 1
            d = 2
            theta = 3
            for j in range(4):
                self.T[i] = [
                    # [0]
                        [cos(self.slovar["DH"][theta][j]), 
                         -cos(self.slovar["DH"][alfa][j])*sin(self.slovar["DH"][theta][j]),
                         sin(self.slovar["DH"][alfa][j])*sin(self.slovar["DH"][theta][j]),
                         self.slovar["DH"][a][j]*cos(self.slovar["DH"][theta][j])],
                    # [1]
                        [sin(self.slovar["DH"][theta][j]), 
                         cos(self.slovar["DH"][alfa][j])*cos(self.slovar["DH"][theta][j]),
                         -sin(self.slovar["DH"][alfa][j])*cos(self.slovar["DH"][theta][j]),
                         self.slovar["DH"][a][j]*sin(self.slovar["DH"][theta][j])],
                    # [2]
                        [0,                 
                         sin(self.slovar["DH"][alfa][j]),
                         cos(self.slovar["DH"][alfa][j]),                  
                         self.slovar["DH"][d][j]],
                    # [3]
                        [0, 0, 0, 1]]
        
    def calculationOfT(self, mode:int) -> array:
        """
        Вычисление матрицы однородного преобразования (Т) (mode 0: от 0 СК до 3 СК (Т_0->3), mode 3: от 3 СК до 0 СК (Т_3->0))
        """
        if mode == 0:
            return matmul(matmul(matmul(self.T[0], self.T[1]), self.T[2]), self.T[3])
        elif mode == 3:
            return matmul(matmul(matmul(self.T[3], self.T[2]), self.T[1]), self.T[0])
        else:
            return "incorrect data"
    
    def calculationOfR(self, mode:int) -> array:
        """
        Вычисление матрицы вращения
        """
        fromT = self.calculationOfT(mode)
        return ([[fromT[0][0], fromT[0][1], fromT[0][2]],
                    [fromT[1][0], fromT[1][1], fromT[1][2]],
                    [fromT[2][0], fromT[2][1], fromT[2][2]]])
    
    def calculationOfP(self, mode:int) -> array:
        """
        Вычисление вектора
        """
        fromT = self.calculationOfT(mode)
        return ([[fromT[0][-1]], [fromT[1][-1]], [fromT[2][-1]]])
    
    def getT(self, num:Union [int, None] = None) -> array:
        """
        Возвращает значения матрицы однородного преобразования по индексу, где индекс соответствует СК
        """
        try: return self.T[num]
        except: 
            print("incorrect data. returned all array <T>:")
            return self.T
        
    def EulersAngles(self, angles: list=[0,0,0]):
        """
        Вычисление углов Эйлера
        """
        R_elera = [
                    # [0]
                    [cos(angles[0])*cos(angles[1])*cos(angles[2]) - sin(angles[0])*sin(angles[2]),
                    -cos(angles[0])*cos(angles[1])*sin(angles[2]) - sin(angles[0])*cos(angles[2]),
                    cos(angles[0])*sin(angles[1])],
                    # [1]
                    [sin(angles[0])*cos(angles[1])*cos(angles[2]) + cos(angles[0])*sin(angles[2]),
                    -sin(angles[0])*cos(angles[1])*sin(angles[2]) + cos(angles[0])*cos(angles[2]),
                    sin(angles[0])*sin(angles[1])],
                    # [2]
                    [-sin(angles[1])*cos(angles[2]),
                    -sin(angles[1])*sin(angles[2]),
                    cos(angles[1])]
                    ]
        return R_elera
    
    @staticmethod
    def print_array(array: list, numOfRound = 0):
        """
        Метод для красивого вывода любого массива (вектора)
        """
        try:
            len(array[0])
            try: 
                len(array[0][0])
                for i in range(len(array[0])):
                    print("|", end='')
                    for j in range(len(array[0][0])):
                        if numOfRound:
                            print(round(array[0][i][j], numOfRound), end=',\t')
                        else:
                            print(array[0][i][j], end=',\t')
                    print("|")
            except:
                for i in range(len(array)):
                    print("|", end='')
                    for j in range(len(array[0])):
                        if numOfRound:
                            print(round(array[i][j], numOfRound), end=',\t')
                        else:
                            print(array[i][j], end=',\t')
                    print("|")
        except:
            print("vector detected")
            print("|", end='')
            for i in range(len(array)):
                if numOfRound:
                    print(array[i])
                    print(round(array[i], numOfRound), end=',\t')
                else:
                    print(array[i], end=',\t')
            print("|")

def main():
    # Ob = baseCalculations()
    # i = 1
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.calculationOfDH([1,1,1,1,1]), 3)
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.get_data("DH"), 3)
    # mode = 0
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.calculationOfT(mode), 3)
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.calculationOfR(mode), 3)
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.calculationOfP(mode), 3)
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.getT(3), 3)
    # print(i)
    # i=i+1
    # print(Ob.getT(), 3)
    # print(i)
    # i=i+1
    # baseCalculations.print_array(Ob.EulersAngles([deg2rad(90),deg2rad(90),deg2rad(90)]), 3)
    pass

if __name__=="__main__":
    main()