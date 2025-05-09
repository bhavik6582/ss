from IPython.display import display, HTML

def main():
    myMessage = 'Transposition Cipher'
    myKey = 10
    ciphertext = encryptMessage(myKey, myMessage)

    print("Cipher Text is:")
    display(HTML(f"<textarea rows='2' cols='50'>{ciphertext}</textarea>"))

def encryptMessage(key, message):
    ciphertext = [''] * key

    for col in range(key):
        position = col
        while position < len(message):
            ciphertext[col] += message[position]
            position += key

    return ''.join(ciphertext)

if __name__ == '__main__':
    main()
