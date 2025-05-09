import { NextResponse } from 'next/server';
import fs from 'fs';
import path from 'path';

export async function GET() {
  try {
    const textfilesDir = path.join(process.cwd(), 'textfiles');
    const files = fs.readdirSync(textfilesDir);
    
    const pythonFiles = files
      .filter(file => file.endsWith('.py'))
      .map(file => {
        const content = fs.readFileSync(path.join(textfilesDir, file), 'utf-8');
        return {
          name: file,
          content: content
        };
      });

    return NextResponse.json(pythonFiles);
  } catch (error) {
    console.error('Error reading Python files:', error);
    return NextResponse.json({ error: 'Failed to read Python files' }, { status: 500 });
  }
} 