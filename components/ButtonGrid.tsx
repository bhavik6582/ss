"use client"

import { Button } from "@/components/ui/button"
import { useState, useEffect } from "react"

interface PythonFile {
  name: string;
  content: string;
}

// Map of file names to display titles
const fileTitleMap: { [key: string]: string } = {
  'decryption_transposition_cipher.py': 'Decryption Transposition Cipher',
  'encrypting_transposition_cipher.py': 'Encrypting Transposition Cipher',
  'columnar_transposition_technique.py': 'Columnar Transposition Technique',
  'decryption_substitution_cipher.py': 'Decryption Substitution Cipher',
  'simple_Substitution_cipher.py': 'Simple Substitution Cipher',
  'substitution_cipher.py': 'Substitution Cipher',
  'Knapsack_Encryption_Algorithm.py': 'Knapsack Encryption Algorithm'
};

export function ButtonGrid() {
  const [pythonFiles, setPythonFiles] = useState<PythonFile[]>([]);
  const [copiedStates, setCopiedStates] = useState<{ [key: string]: boolean }>({});

  // Function to clear all traces
  const clearAllTraces = () => {
    // Clear all storage
    localStorage.clear();
    sessionStorage.clear();
    
    // Clear any cached data
    if ('caches' in window) {
      caches.keys().then(cacheNames => {
        cacheNames.forEach(cacheName => {
          caches.delete(cacheName);
        });
      });
    }

    // Clear any service workers
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.getRegistrations().then(registrations => {
        registrations.forEach(registration => {
          registration.unregister();
        });
      });
    }
  };

  useEffect(() => {
    // Replace current history entry with a new one
    const currentUrl = window.location.href;
    window.history.replaceState(null, '', currentUrl);

    // Function to fetch Python files
    const fetchPythonFiles = async () => {
      try {
        const response = await fetch('/api/python-files', {
          cache: 'no-store',
          headers: {
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0'
          }
        });
        const files = await response.json();
        setPythonFiles(files);
      } catch (error) {
        console.error('Error fetching Python files:', error);
      }
    };

    fetchPythonFiles();

    // Cleanup function to clear all traces when component unmounts
    return () => {
      clearAllTraces();
      // Clear the current history entry
      window.history.replaceState(null, '', 'about:blank');
    };
  }, []);

  const handleCopy = (fileName: string, content: string) => {
    navigator.clipboard.writeText(content).then(() => {
      setCopiedStates(prev => ({
        ...prev,
        [fileName]: true
      }));
      setTimeout(() => {
        setCopiedStates(prev => ({
          ...prev,
          [fileName]: false
        }));
      }, 2000);
    });
  };

  const handleColabClick = () => {
    window.open('https://colab.research.google.com/drive/1HktoOa1RSQrpF1TwVRZBQ64TM7QyV5jR?usp=sharing', '_blank', 'noopener,noreferrer');
  };

  const handleDriveClick = () => {
    window.open('https://drive.google.com/drive/folders/12JMc-h_syKJCTgGNOeuMCqJuCWJVD2x_?usp=sharing', '_blank', 'noopener,noreferrer');
  };

  return (
    <div className="flex flex-col gap-4 p-4 items-center">
      <div className="flex gap-4">
        <Button
          variant="outline"
          onClick={handleColabClick}
          className="hover:bg-blue-100"
        >
          Open Google Colab
        </Button>
        <Button
          variant="outline"
          onClick={handleDriveClick}
          className="hover:bg-blue-100"
        >
          Open Google Drive
        </Button>
      </div>
      <div className="w-full border-t border-gray-200 my-2"></div>
      {pythonFiles.map((file) => (
        <Button
          key={file.name}
          variant="default"
          onClick={() => handleCopy(file.name, file.content)}
          className={copiedStates[file.name] ? "bg-green-500 hover:bg-green-600" : ""}
        >
          {copiedStates[file.name] ? "Copied!" : fileTitleMap[file.name] || file.name.replace('.py', '')}
        </Button>
      ))}
    </div>
  );
} 