import { Outlet } from 'react-router-dom'
import './Layout.css'

export const Layout = () => {
  return (
    <div className="layout">
      <header className="layout-header">
        <div className="container">
          <h1 className="logo">Mr.Chef Assistant</h1>
        </div>
      </header>

      <main className="layout-main">
        <Outlet />
      </main>

      <footer className="layout-footer">
        <div className="container">
          <p>&copy; 2025 Doosan Robotics Material Processing System</p>
        </div>
      </footer>
    </div>
  )
}
