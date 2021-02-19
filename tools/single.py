class Singleton(type):
    _instance = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            print('call')
            cls._instance[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

# class T(metaclas=Singleton):  # test of singleton metaclass
#  def __init__(self):
#     print('init')
