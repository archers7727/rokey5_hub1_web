import type { Metadata } from 'next'
import './globals.css'

export const metadata: Metadata = {
  title: 'Rokey Robot Hub',
  description: 'Robot Material Processing Hub',
}

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode
}>) {
  return (
    <html lang="ko">
      <body>{children}</body>
    </html>
  )
}
