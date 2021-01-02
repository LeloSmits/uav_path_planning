def test1(a, b):
    return a + b


def test2(func, x1, x2):
    return func(x1, x2)


print(test2(test1, 3, 4))
