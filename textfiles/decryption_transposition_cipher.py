import math


def main():
    myMessage = 'Toners raiCntisippoh'
    myKey = 6
    plaintext = decryptMessage(myKey, myMessage)
   
    print("The plain text is")
    print(plaintext)


def decryptMessage(key, message):
    numOfColumns = math.ceil(len(message) / key)
    numOfRows = key
    numOfShadedBoxes = (numOfColumns * numOfRows) - len(message)
   
    # Create a list of empty strings for each column
    plaintext = [''] * numOfColumns
   
    col = 0
    row = 0
   
    # Go through the message, one symbol at a time
    for symbol in message:
        plaintext[col] += symbol
        col += 1
        # When reaching the end of a column or skipping shaded boxes
        if (col == numOfColumns) or (col == numOfColumns - 1 and row >= numOfRows - numOfShadedBoxes):
            col = 0
            row += 1
   
    return ''.join(plaintext)


if __name__ == '__main__':
    main()
