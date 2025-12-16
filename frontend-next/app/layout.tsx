import type { Metadata } from 'next'
import Link from 'next/link'
import './globals.css'

export const metadata: Metadata = {
  title: 'Mr.Chef Assistant - Rokey Robot Hub',
  description: 'Robot Material Processing Hub with Next.js + Supabase',
}

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode
}>) {
  return (
    <html lang="ko">
      <body className="min-h-screen bg-gray-50">
        <div className="flex flex-col min-h-screen">
          <header className="bg-white shadow-sm border-b border-gray-200">
            <div className="container mx-auto px-4 py-4">
              <Link href="/dashboard">
                <h1 className="text-2xl font-bold text-gray-900 cursor-pointer hover:text-blue-600 transition-colors">
                  Mr.Chef Assistant
                </h1>
              </Link>
            </div>
          </header>

          <main className="flex-1 container mx-auto px-4 py-8">
            {children}
          </main>

          <footer className="bg-white border-t border-gray-200">
            <div className="container mx-auto px-4 py-4">
              <p className="text-sm text-gray-600 text-center">
                &copy; 2025 Doosan Robotics Material Processing System
              </p>
            </div>
          </footer>
        </div>
      </body>
    </html>
  )
}
