class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            print('call')
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


# class T:  # test of singleton metaclass
#     __metaclass__ = Singleton
#     def __init__(self):
#         print("init")
