import doctest

class bounded_subsets:
    def __init__(self, s , c :int):
        self.s = sorted(s)
        # #remove nums that are bigger than c -we arw dealling with positiv nums
        # for i in s :
        #     if i > c:
        #         s.remove(i)
        self.length = len(s)
        self.indexes = [] # saving indexes for creating sub list
        self.c = c
        self.sub_size = 0 # represent the sublist size

    def __iter__(self):
        return self

    def __next__(self):
        """
        This function returns the next subList with sum smaller then c

        >>> myit = bounded_subsets((1, 2, 3),2)
        >>> print(next(myit))
        []
        >>> print(next(myit))
        [1]
        >>> for i in bounded_subsets([1,2,3],4): print(i)
        []
        [1]
        [2]
        [3]
        [1, 2]
        [1, 3]
        >>> myit2 = bounded_subsets((),2)
        >>> print(next(myit2))
        []
        >>> print(next(myit2))
        []

        >>> myit3 = bounded_subsets((1,4,9),0)
        >>> print(next(myit3))
        []

        """
        if self.length == 0:
            return []
        if self.sub_size == 0:
            self.sub_size += 1
            self.indexes.append(0)
            return []

        while True:
            try:
                res = [self.s[i] for i in self.indexes]
            except:
                # update indexes
                self.update()
                continue
            if self.sub_size > self.length:
                raise StopIteration
            # if sublist is the list itself
            if self.sub_size == self.length:
                if sum(res) > self.c:
                    raise StopIteration
                else:
                    self.sub_size += 1
                    return res
            if sum(res) <= self.c:
                break
            else:
                self.update()
        self.update()
        return res

    def update(self):

        if not (self.indexes[self.sub_size - 1] >= (self.length - 1)):
            self.indexes[self.sub_size - 1] += 1
        else:
            if self.sub_size == 1:
                self.indexes[0] = 0
                self.indexes.append(1)
                self.sub_size += 1
                return
            end = self.sub_size - 1
            flag = True
            while end > 0:
                if self.indexes[end] == (self.indexes[end - 1] + 1):
                    end -= 1
                    break
                else:
                    flag = False
                    end -= 1
            if flag:
                for i in range(self.sub_size):
                    self.indexes[i] = i
                self.indexes.append(self.sub_size)
                self.sub_size += 1
                return
            self.indexes[end] += 1
            temp = self.indexes[end] + 1
            for i in range(end + 1, self.sub_size):
                self.indexes[i] = temp
                temp += 1
            return

if __name__ == '__main__':
    for s in zip(range(5), bounded_subsets(range(100), 1000000000000)):
        print(s)
    for s in bounded_subsets([1,2,3],4):
        print(s)
    for s in bounded_subsets(range(50,150), 103):
        print(s)
    mytuple = (1, 2, 3)
    myit = bounded_subsets(mytuple,2)
    print(next(myit))
    print(next(myit))
    print(next(myit))
    (failures, tests) = doctest.testmod(report=True, optionflags=doctest.NORMALIZE_WHITESPACE + doctest.ELLIPSIS)
    print("{} failures, {} tests".format(failures, tests))