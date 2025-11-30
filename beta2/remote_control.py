"""
Remote control interface for manual override and emergency stop.
Provides network interface for external control commands.
"""

import socket
import threading
import json
import time

class RemoteControl:
    """Network interface for remote manual override commands."""
    
    def __init__(self, port=8888, host='0.0.0.0'):
        """
        Initialize remote control server.
        
        Args:
            port: TCP port to listen on (default: 8888)
            host: Host address to bind to (default: '0.0.0.0' for all interfaces)
        """
        self.port = port
        self.host = host
        self.socket = None
        self.server_thread = None
        self.running = False
        self.callbacks = {
            'emergency_stop': None,
            'manual_override': None,
            'status_request': None
        }
    
    def set_emergency_stop_callback(self, callback):
        """Set callback function for emergency stop command."""
        self.callbacks['emergency_stop'] = callback
    
    def set_manual_override_callback(self, callback):
        """Set callback function for manual override commands."""
        self.callbacks['manual_override'] = callback
    
    def set_status_request_callback(self, callback):
        """Set callback function for status requests."""
        self.callbacks['status_request'] = callback
    
    def start_server(self):
        """Start the remote control server in a separate thread."""
        if self.running:
            print("[REMOTE] Server already running")
            return
        
        self.running = True
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()
        print(f"[REMOTE] Remote control server started on {self.host}:{self.port}")
        print("[REMOTE] Commands: 'emergency_stop', 'status', or JSON commands")
    
    def stop_server(self):
        """Stop the remote control server."""
        self.running = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        print("[REMOTE] Remote control server stopped")
    
    def _server_loop(self):
        """Main server loop - handles incoming connections."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            self.socket.settimeout(1.0)  # Allow periodic checking of self.running
            
            while self.running:
                try:
                    client_socket, address = self.socket.accept()
                    print(f"[REMOTE] Connection from {address}")
                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_socket, address),
                        daemon=True
                    )
                    client_thread.start()
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"[REMOTE] Error accepting connection: {e}")
        except Exception as e:
            if self.running:
                print(f"[REMOTE] Server error: {e}")
    
    def _handle_client(self, client_socket, address):
        """Handle a single client connection."""
        try:
            client_socket.settimeout(5.0)
            while self.running:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                
                data = data.strip()
                response = self._process_command(data)
                
                if response:
                    client_socket.sendall((response + '\n').encode('utf-8'))
                
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[REMOTE] Error handling client {address}: {e}")
        finally:
            client_socket.close()
            print(f"[REMOTE] Client {address} disconnected")
    
    def _process_command(self, command):
        """Process incoming command and return response."""
        command = command.strip().lower()
        
        # Emergency stop command
        if command == 'emergency_stop' or command == 'kill' or command == 'stop':
            print("[REMOTE] Emergency stop command received!")
            if self.callbacks['emergency_stop']:
                self.callbacks['emergency_stop']()
            return "OK: Emergency stop activated"
        
        # Status request
        elif command == 'status' or command == 'info':
            if self.callbacks['status_request']:
                status = self.callbacks['status_request']()
                return json.dumps(status)
            return "OK: Status requested"
        
        # JSON command (for future expansion)
        elif command.startswith('{'):
            try:
                cmd_data = json.loads(command)
                cmd_type = cmd_data.get('command', '').lower()
                
                if cmd_type == 'manual_override':
                    if self.callbacks['manual_override']:
                        self.callbacks['manual_override'](cmd_data.get('data', {}))
                    return "OK: Manual override command processed"
                else:
                    return f"ERROR: Unknown command type: {cmd_type}"
            except json.JSONDecodeError:
                return "ERROR: Invalid JSON"
        
        else:
            return f"ERROR: Unknown command: {command}. Use 'emergency_stop', 'status', or JSON"
    
    def send_command(self, host, port, command):
        """
        Send a command to a remote control server (client mode).
        
        Args:
            host: Server hostname or IP
            port: Server port
            command: Command string to send
        
        Returns:
            str: Server response
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))
            sock.sendall((command + '\n').encode('utf-8'))
            response = sock.recv(1024).decode('utf-8').strip()
            sock.close()
            return response
        except Exception as e:
            return f"ERROR: {e}"

