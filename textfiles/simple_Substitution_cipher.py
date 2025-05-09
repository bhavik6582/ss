import random


LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'


def main():
    message = input("Enter your message: ")
    mode = input("E for Encrypt, D for Decrypt: ").upper()
    key = ''


    while checkKey(key) is False:
        key = input("Enter 26 ALPHA key (leave blank for random key): ").upper()
        if key == '':
            key = getRandomKey()
        if checkKey(key) is False:
            print('There is an error in the key or symbol set.')


    translated = translateMessage(message, key, mode)
    print('Using key: %s' % (key))
    print('Result: ' + translated)


def checkKey(key):
    keyString = ''.join(sorted(list(key)))
    return keyString == LETTERS


def translateMessage(message, key, mode):
    translated = ''
    charsA = LETTERS
    charsB = key


    if mode == 'D':
        charsA, charsB = charsB, charsA


    for symbol in message:
        if symbol.upper() in charsA:
            symIndex = charsA.find(symbol.upper())
            if symbol.isupper():
                translated += charsB[symIndex].upper()
            else:
                translated += charsB[symIndex].lower()
        else:
            translated += symbol
    return translated


def getRandomKey():
    randomList = list(LETTERS)
    random.shuffle(randomList)
    return ''.join(randomList)


if __name__ == '__main__':
    main()
