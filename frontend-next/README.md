# Rokey Robot Hub - Next.js Frontend

This is the Next.js frontend for the Rokey Robot Hub, migrated from React + Vite to Next.js + Supabase.

## Getting Started

### Prerequisites

- Node.js 18+ installed
- Supabase account and project created
- Environment variables configured

### Installation

1. Install dependencies:

```bash
npm install
```

2. Copy `.env.example` to `.env.local` and fill in your Supabase credentials:

```bash
cp .env.example .env.local
```

3. Update `.env.local` with your Supabase project details:

```env
NEXT_PUBLIC_SUPABASE_URL=https://your-project.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=your-anon-key
SUPABASE_SERVICE_ROLE_KEY=your-service-role-key
```

### Development

Run the development server:

```bash
npm run dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

### Building for Production

```bash
npm run build
npm start
```

## Project Structure

```
frontend-next/
├── app/                    # Next.js App Router pages
│   ├── api/               # API Routes (serverless functions)
│   ├── layout.tsx         # Root layout
│   └── page.tsx           # Home page
├── components/            # React components
├── hooks/                 # Custom React hooks (including Realtime)
├── lib/
│   └── supabase/         # Supabase client configuration
├── types/                # TypeScript type definitions
└── public/               # Static files
```

## Key Features

- **Next.js App Router**: Modern React framework with server-side rendering
- **Supabase**: PostgreSQL database with real-time capabilities
- **Supabase Realtime**: WebSocket-based real-time updates
- **API Routes**: Serverless functions for backend logic
- **TypeScript**: Type-safe development
- **Tailwind CSS**: Utility-first CSS framework

## API Routes

- `GET /api/materials` - Get all materials
- `GET /api/modes?material=<id>` - Get modes (optionally filtered by material)
- `GET /api/jobs` - Get job history
- `POST /api/jobs` - Create new task
- `GET /api/robot/state` - Get current robot state

## Realtime Hooks

- `useTasksRealtime()` - Subscribe to task queue updates
- `useRobotStateRealtime()` - Subscribe to robot state updates

## Deployment

This project is configured for deployment on Vercel:

1. Push your code to GitHub
2. Import the project on Vercel
3. Configure environment variables in Vercel dashboard
4. Deploy!

See [MIGRATION_TO_VERCEL_SUPABASE.md](../MIGRATION_TO_VERCEL_SUPABASE.md) for detailed migration guide.

## Learn More

- [Next.js Documentation](https://nextjs.org/docs)
- [Supabase Documentation](https://supabase.com/docs)
- [Vercel Documentation](https://vercel.com/docs)
