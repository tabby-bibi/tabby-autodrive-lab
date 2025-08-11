
class CustomError(Exception):
    def __init__(self, message: str):
        self.message = message

    def __str__(self):
        return self.msg

    # TODO : 일단 예시파일임 나중에 알아보고 정의할 예정임