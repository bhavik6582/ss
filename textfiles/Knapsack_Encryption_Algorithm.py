import random


# Function to generate a super-increasing sequence for the public key
def generate_super_increasing_sequence(n):
    sequence = [random.randint(1, 10)]
    while len(sequence) < n:
        next_element = sum(sequence) + random.randint(1, 10)
        sequence.append(next_element)
    return sequence


# Function to generate the private key from the public key
def generate_private_key(public_key, q, r):
    private_key = [(r * element) % q for element in public_key]
    return private_key


# Function to encrypt the plaintext using the public key
def knapsack_encrypt(plaintext, public_key):
    encrypted_message = sum(public_key[i] for i in range(len(plaintext)) if plaintext[i] == '1')
    return encrypted_message


# Function to decrypt the ciphertext using the private key
def knapsack_decrypt(ciphertext, private_key, q, r):
    r_inverse = pow(r, -1, q)  # Modular multiplicative inverse of r
    s = (ciphertext * r_inverse) % q


    decrypted_message = ''
    for element in reversed(private_key):
        if s >= element:
            decrypted_message = '1' + decrypted_message
            s -= element
        else:
            decrypted_message = '0' + decrypted_message
    return decrypted_message


# Example usage
if __name__ == "__main__":
    n = 8  # Number of elements in the super-increasing sequence
    q = 2000  # Modulus (must be > sum of super-increasing sequence)
    r = 31    # Multiplier for generating private key (must be coprime with q)


    # Generate super-increasing sequence (private key base)
    super_increasing_seq = generate_super_increasing_sequence(n)
   
    # Compute public and private keys
    public_key = generate_private_key(super_increasing_seq, q, r)
    private_key = super_increasing_seq  # Actual private key is the original super-increasing sequence


    plaintext = "11001010"
    ciphertext = knapsack_encrypt(plaintext, public_key)
    decrypted_message = knapsack_decrypt(ciphertext, private_key, q, r)


    print("Super-increasing sequence (Private Key base):", super_increasing_seq)
    print("Public Key:", public_key)
    print("Original Message:", plaintext)
    print("Encrypted Ciphertext:", ciphertext)
    print("Decrypted Message:", decrypted_message)
